from dronekit import connect
from pymavlink import mavutil
import time

# === CONFIG ===
PORT = '/dev/ttyUSB0'  # sau '/dev/ttyUSB0' dacă e pe USB
BAUD = 57600

print("[INFO] Conectare la Pixhawk...")
vehicle = connect(PORT, baud=BAUD, wait_ready=False)
time.sleep(1)

print("[INFO] Trimit comanda de armare forțată (MAV_CMD_COMPONENT_ARM_DISARM)...")
vehicle._master.mav.command_long_send(
    vehicle._master.target_system,
    vehicle._master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,          # confirmation
    1,          # param1: 1=arm, 0=disarm
    21196,      # param2: magic code pentru override
    0, 0, 0, 0, 0
)

# Așteaptă să confirme
for i in range(10):
    if vehicle.armed:
        print("[✅] Drone ARMED!")
        break
    print(f"[...] Aștept confirmare... ({i+1}/10)")
    time.sleep(1)
else:
    print("[❌] Armarea a eșuat complet (chiar și forțat).")

vehicle.close()
