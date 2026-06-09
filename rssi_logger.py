import subprocess
import time
import sys
import csv
import datetime

# --- CONFIGURATION ---
TARGET_SSID = "RASPI-AP"  # Pi1's Hotspot Name
INTERFACE = "wlan0"

def get_rssi():
    try:
        cmd = f"sudo iw dev {INTERFACE} scan | grep -i {TARGET_SSID} -B 5 | grep signal"
        result = subprocess.check_output(cmd, shell=True, text=True)
        if "signal:" in result:
            parts = result.split("signal:")
            return float(parts[1].split("dBm")[0].strip())
    except:
        return None
    return None

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: sudo python3 rssi_logger.py <filename.csv>")
        sys.exit(1)

    filename = sys.argv[1]
    print(f"Logging {TARGET_SSID} to {filename}...")
    
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "RSSI"])
        try:
            while True:
                rssi = get_rssi()
                if rssi is not None:
                    print(f"Signal: {rssi} dBm")
                    writer.writerow([datetime.datetime.now().strftime("%H:%M:%S"), rssi])
                    file.flush()
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nDone.")
