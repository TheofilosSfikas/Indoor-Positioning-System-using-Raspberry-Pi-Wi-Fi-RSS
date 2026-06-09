import socket
import subprocess
import time

# --- CONFIGURATION ---
ANCHOR_NAME = "Pi3"
SERVER_IP = "192.168.50.1" 
SERVER_PORT = 5000
INTERFACE = "wlan0"
TARGET_SSID = "RASPI-AP"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

print(f"--- {ANCHOR_NAME} ROBUST SENDER ---")

def get_rssi():
    try:
        # We use 'timeout' command in Linux to kill the scan if it hangs > 1s
        cmd = f"sudo timeout 1s iw dev {INTERFACE} scan | grep -i {TARGET_SSID} -B 5 | grep signal"
        
        # Run command
        result = subprocess.check_output(cmd, shell=True, text=True)
        
        if "signal:" in result:
            # Parse "-45.00 dBm"
            return float(result.split("signal:")[1].split("dBm")[0].strip())
    except subprocess.CalledProcessError:
        # This happens if timeout kills the process (Good! It prevents hanging)
        return None
    except Exception as e:
        return None
    return None

while True:
    start_time = time.time()
    rssi = get_rssi()
    
    if rssi is not None:
        msg = f"{ANCHOR_NAME}:{rssi}"
        sock.sendto(msg.encode(), (SERVER_IP, SERVER_PORT))
        print(f"Sent: {rssi} dBm (Took {time.time() - start_time:.2f}s)")
    else:
        print("Scan skipped/timed out")
        
    # We don't sleep here because the scan itself acts as the delay
