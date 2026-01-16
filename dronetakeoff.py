import time
from dronekit import connect, VehicleMode

# 1. Connect to the vehicle
# Use the port you defined in your MAVProxy command: --out 127.0.0.1:14551
print("Connecting to vehicle...")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks...")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    
    # Wait for mode to change
    while vehicle.mode != 'GUIDED':
        print(" Waiting for GUIDED mode...")
        time.sleep(0.5)
    
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off! Target altitude: {target_altitude}m")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Current Altitude: {current_alt:.1f}m")
        # Break when within 5% of target altitude
        if current_alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# --- EXECUTION ---

# Take off to 10 meters
arm_and_takeoff(10)

# Hover for 5 seconds
print("Hovering...")
time.sleep(5)

# Land
print("Setting mode to LAND...")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object before exiting script
print("Closing vehicle connection")
vehicle.close()