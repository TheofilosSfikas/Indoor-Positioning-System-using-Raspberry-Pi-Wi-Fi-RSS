#!/usr/bin/env python3
import socket
import math
import time
import threading

# --- NETWORK CONFIG ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5000
LAPTOP_IP = "192.168.50.156"
LAPTOP_PORT = 6000

# --- TUNING ---
SEND_HZ = 10.0               # Engine loop frequency (Hz)
STALE_SEC = 2.5              # Max age of RSSI data (s)
STEP = 0.05                  # Grid resolution (m)
ERROR_LEVELS = [0.46, 0.80, 1.50, 3.00]
MAX_SPEED = 2.0              # m/s, human max speed
EPS = 1e-6
MIN_CLUSTER_SIZE = 2
VAR_FLOOR = 0.20             # Minimum measurement variance

# --- CALIBRATION (RSSI -> distance) ---
CALIBRATION = {
    "Pi2": {"A": -46.0, "n": 3.1},
    "Pi3": {"A": -47.0, "n": 3.7},
    "Pi4": {"A": -45.2, "n": 3.0}  #have tried n=4.2,2.6,2.8 best,
}

# --- ROOM GEOMETRY (meters) ---
ROOM_W = 3.43
ROOM_H = 2.25
ANCHORS = {
    "Pi2": (0.0, 0.0),
    "Pi3": (0.0, 1.90),
    "Pi4": (3.43, 0.08)
}

# ---------------- Matrix helpers (pure Python) ----------------
def mat_mul(A, B):
    rows_A, cols_A = len(A), len(A[0])
    cols_B = len(B[0])
    C = [[0.0] * cols_B for _ in range(rows_A)]
    for i in range(rows_A):
        row_A = A[i]
        for j in range(cols_B):
            s = 0.0
            for k in range(cols_A):
                s += row_A[k] * B[k][j]
            C[i][j] = s
    return C

def mat_add(A, B):
    return [[a + b for a, b in zip(rA, rB)] for rA, rB in zip(A, B)]

def mat_sub(A, B):
    return [[a - b for a, b in zip(rA, rB)] for rA, rB in zip(A, B)]

def mat_trans(A):
    return [list(row) for row in zip(*A)]

def mat_id(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]

def mat_scale(A, s):
    return [[x * s for x in row] for row in A]

def mat_inv_2x2(M):
    a, b = M[0]
    c, d = M[1]
    det = a * d - b * c
    if abs(det) < 1e-12:
        return [[0.0, 0.0], [0.0, 0.0]]
    inv = 1.0 / det
    return [[d * inv, -b * inv], [-c * inv, a * inv]]

# ---------------- Kalman Filter ----------------
class Kalman2D:
    def __init__(self, dt):
        self.dt = dt
        self.x = [[ROOM_W / 2.0], [ROOM_H / 2.0], [0.0], [0.0]]
        self.F = [[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]]
        self.H = [[1, 0, 0, 0],
                  [0, 1, 0, 0]]
        self.P = mat_scale(mat_id(4), 10.0)
        self.Q = [[0.05, 0,    0,   0],
                  [0,    0.05, 0,   0],
                  [0,    0,    0.2, 0],
                  [0,    0,    0,   0.2]]

    def predict(self):
        self.x = mat_mul(self.F, self.x)
        FP = mat_mul(self.F, self.P)
        self.P = mat_add(mat_mul(FP, mat_trans(self.F)), self.Q)

    def update(self, z, varx, vary):
        R = [[varx, 0], [0, vary]]
        z_vec = [[z[0]], [z[1]]]
        y = mat_sub(z_vec, mat_mul(self.H, self.x))
        S = mat_add(mat_mul(mat_mul(self.H, self.P), mat_trans(self.H)), R)
        K = mat_mul(mat_mul(self.P, mat_trans(self.H)), mat_inv_2x2(S))
        self.x = mat_add(self.x, mat_mul(K, y))
        I_KH = mat_sub(mat_id(4), mat_mul(K, self.H))
        self.P = mat_mul(I_KH, self.P)

        # Clamp internal state to room bounds
        self.x[0][0] = max(0.0, min(ROOM_W, self.x[0][0]))
        self.x[1][0] = max(0.0, min(ROOM_H, self.x[1][0]))

        return self.x[0][0], self.x[1][0]

# ---------------- Shared state ----------------
sock_listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_listen.bind((UDP_IP, UDP_PORT))
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

distances = {"Pi2": None, "Pi3": None, "Pi4": None}
times = {"Pi2": 0.0, "Pi3": 0.0, "Pi4": 0.0}
lock = threading.Lock()

last_processed = 0.0
last_good_radii = {"Pi2": 0.0, "Pi3": 0.0, "Pi4": 0.0}

def get_distance(name, rssi):
    if name not in CALIBRATION:
        return None
    c = CALIBRATION[name]
    return 10 ** ((c["A"] - rssi) / (10 * c["n"]))

# ---------------- Trilateration Grid Search ----------------
def calculate_grid_raw(snap):
    d2, d3, d4 = snap["Pi2"], snap["Pi3"], snap["Pi4"]
    if d2 is None or d3 is None or d4 is None:
        return None, None, None, None, None, 0

    for current_error in ERROR_LEVELS:
        valid_points = []
        x = 0.0
        while x < ROOM_W + 1e-6:
            y = 0.0
            while y < ROOM_H + 1e-6:
                d_p2 = math.hypot(x - ANCHORS["Pi2"][0], y - ANCHORS["Pi2"][1])
                if abs(d_p2 - d2) <= current_error:
                    d_p3 = math.hypot(x - ANCHORS["Pi3"][0], y - ANCHORS["Pi3"][1])
                    if abs(d_p3 - d3) <= current_error:
                        d_p4 = math.hypot(x - ANCHORS["Pi4"][0], y - ANCHORS["Pi4"][1])
                        if abs(d_p4 - d4) <= current_error:
                            valid_points.append((x, y))
                y += STEP
            x += STEP

        if not valid_points:
            continue

        def to_idx(px, py):
            return int(round(px / STEP)), int(round(py / STEP))

        grid_map = {to_idx(px, py): (px, py) for (px, py) in valid_points}
        visited = set()
        clusters = []

        for idx in list(grid_map.keys()):
            if idx in visited:
                continue
            stack = [idx]
            comp = []
            visited.add(idx)
            while stack:
                cur = stack.pop()
                comp.append(grid_map[cur])
                cx, cy = cur
                for nei in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if nei in grid_map and nei not in visited:
                        visited.add(nei)
                        stack.append(nei)
            clusters.append(comp)

        if not clusters:
            continue

        clusters.sort(key=lambda c: len(c), reverse=True)
        largest = clusters[0]
        cluster_size = len(largest)

        if cluster_size < MIN_CLUSTER_SIZE:
            continue

        wx_sum = wy_sum = w_sum = 0.0
        for (px, py) in largest:
            r2 = abs(math.hypot(px - ANCHORS["Pi2"][0], py - ANCHORS["Pi2"][1]) - d2)
            r3 = abs(math.hypot(px - ANCHORS["Pi3"][0], py - ANCHORS["Pi3"][1]) - d3)
            r4 = abs(math.hypot(px - ANCHORS["Pi4"][0], py - ANCHORS["Pi4"][1]) - d4)
            resid = r2 + r3 + r4
            w = 1.0 / (resid + EPS)
            wx_sum += px * w
            wy_sum += py * w
            w_sum += w

        raw_x = wx_sum / w_sum if w_sum > 0 else ROOM_W / 2.0
        raw_y = wy_sum / w_sum if w_sum > 0 else ROOM_H / 2.0

        if cluster_size > 1:
            mean_x = sum(p[0] for p in largest) / cluster_size
            mean_y = sum(p[1] for p in largest) / cluster_size
            var_x = sum((p[0] - mean_x) ** 2 for p in largest) / (cluster_size - 1)
            var_y = sum((p[1] - mean_y) ** 2 for p in largest) / (cluster_size - 1)
        else:
            var_x = var_y = 0.25

        error_penalty = (current_error ** 2) * 0.25
        var_x = max(VAR_FLOOR, min(var_x + error_penalty, 4.0))
        var_y = max(VAR_FLOOR, min(var_y + error_penalty, 4.0))

        return raw_x, raw_y, var_x, var_y, current_error, cluster_size

    return None, None, None, None, None, 0

# ---------------- RSSI listener thread ----------------
def rssi_thread():
    print("[Thread] Listening for UDP packets...")
    while True:
        try:
            data, _ = sock_listen.recvfrom(1024)
            msg = data.decode().strip()
            if ":" not in msg:
                continue

            name, val = msg.split(":", 1)
            try:
                rssi = float(val)
            except ValueError:
                continue

            dist = get_distance(name, rssi)
            if dist is None:
                continue

            with lock:
                distances[name] = dist
                times[name] = time.time()

        except Exception:
            time.sleep(0.01)

# ---------------- MAIN LOOP ----------------
if __name__ == "__main__":
    kf = Kalman2D(dt=1.0 / SEND_HZ)
    threading.Thread(target=rssi_thread, daemon=True).start()

    print(f"--- Pi1 Brain Engine ({SEND_HZ} Hz) ---")
    interval = 1.0 / SEND_HZ

    while True:
        start_time = time.time()

        # 1) Predict
        kf.predict()

        # 2) Snapshot current shared state
        with lock:
            snap_dist = dict(distances)
            snap_time = dict(times)

        # 3) Freshness and synchronization gate
        now = time.time()
        valid = all(snap_dist[a] is not None for a in snap_dist)
        fresh = valid and all((now - snap_time[a]) <= STALE_SEC for a in snap_time)
        new_trio = fresh and (min(snap_time.values()) > last_processed)

        raw_x = raw_y = var_x = var_y = None
        err = clus = None

        if new_trio:
            # Mark this trio as processed immediately so it is never re-run
            last_processed = max(snap_time.values())

            raw_x, raw_y, var_x, var_y, err, clus = calculate_grid_raw(snap_dist)

            if raw_x is not None:
                prev_x, prev_y = kf.x[0][0], kf.x[1][0]
                d = math.hypot(raw_x - prev_x, raw_y - prev_y)
                if d > (MAX_SPEED * interval) and d > 0.1:
                    factor = (MAX_SPEED * interval) / d
                    raw_x = prev_x + (raw_x - prev_x) * factor
                    raw_y = prev_y + (raw_y - prev_y) * factor

                smooth_x, smooth_y = kf.update([raw_x, raw_y], var_x, var_y)
                print(
                    f"[NEW DATA] Cloud:{clus} Err:{err:.2f} | "
                    f"Var:{var_x:.2f},{var_y:.2f} | "
                    f"Raw:{raw_x:.2f},{raw_y:.2f} | "
                    f"Smooth:{smooth_x:.2f},{smooth_y:.2f}"
                )
            else:
                smooth_x, smooth_y = kf.x[0][0], kf.x[1][0]
                print("[NEW DATA] No valid cluster found - coasting.")
        else:
            kf.x[2][0] *= 0.5
            kf.x[3][0] *= 0.5
            smooth_x, smooth_y = kf.x[0][0], kf.x[1][0]
            print("[COAST] No new complete data.")

        # 4) Send to laptop
        sx = max(0.0, min(ROOM_W, smooth_x))
        sy = max(0.0, min(ROOM_H, smooth_y))

        try:
            sock_send.sendto(f"POS:{sx:.2f},{sy:.2f}".encode(), (LAPTOP_IP, LAPTOP_PORT))

            if fresh:
                last_good_radii["Pi2"] = snap_dist["Pi2"]
                last_good_radii["Pi3"] = snap_dist["Pi3"]
                last_good_radii["Pi4"] = snap_dist["Pi4"]

            sock_send.sendto(
                f"RAD:{last_good_radii['Pi2']:.2f},{last_good_radii['Pi3']:.2f},{last_good_radii['Pi4']:.2f}".encode(),
                (LAPTOP_IP, LAPTOP_PORT)
            )
        except Exception:
            pass

        # 5) Sleep to maintain loop rate
        elapsed = time.time() - start_time
        if elapsed < interval:
            time.sleep(interval - elapsed)
