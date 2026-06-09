import socket
import math
import time

# --- YOUR CALIBRATED PHYSICS CONSTANTS ---
A = -31.11   # Signal at 1m
n = 5.5      # Path Loss Exponent (Your Room's Constant)

# --- NETWORK CONFIG ---
UDP_IP = "0.0.0.0" # Listen to ALL incoming traffic
UDP_PORT = 5000

# --- MATH FUNCTION ---
def calculate_distance(rssi):
    # D = 10 ^ ((A - RSSI) / (10 * n))
    exponent = (A - rssi) / (10 * n)
    return 10 ** exponent

# --- SETUP SOCKET ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"--- Pi1 LISTENING ON PORT {UDP_PORT} ---")
print(f"Using Constants: A={A}, n={n}")
print("Waiting for Anchors...")

# --- MAIN LOOP ---
try:
    while True:
        # 1. Receive Data (Blocking wait)
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        message = data.decode().strip()
        
        # Message format expected: "Pi2:-45.0"
        if ":" in message:
            anchor_name, rssi_str = message.split(":")
            rssi = float(rssi_str)
            
            # 2. Calculate Distance
            dist = calculate_distance(rssi)
            
            # 3. Print Live Result
            print(f"[{anchor_name}] Signal: {rssi} dBm --> Distance: {dist:.2f}m")

except KeyboardInterrupt:
    print("\nServer Stopped.")
    sock.close()
