import math

# --- PI3 CONSTANTS ---
A = -41.42   # Signal strength at 1m
n = 5.0      # Path loss exponent

def get_dist(rssi):
    # Formula: Distance = 10 ^ ((A - RSSI) / (10 * n))
    exponent = (A - rssi) / (10 * n)
    return 10 ** exponent

# --- PI3 MEASURED DATA ---
measured_rssis = [-22.49, -41.42, -48.86, -65.68]
actual_meters  = [0.5,    1.0,    2.0,    3.0]

print(f"Testing Pi3 with A={A}, n={n}")
print(f"{'RSSI':<10} | {'Calc Dist':<10} | {'Real Dist':<10} | {'Error'}")
print("-" * 50)

total_error = 0
for i in range(len(measured_rssis)):
    rssi = measured_rssis[i]
    real = actual_meters[i]
    calc = get_dist(rssi)
    error = abs(calc - real)
    total_error += error
    
    print(f"{rssi:<10} | {calc:<10.2f} | {real:<10.1f} | {error:.2f}m")

print("-" * 50)
print(f"Average Error: {total_error/4:.2f}m")
