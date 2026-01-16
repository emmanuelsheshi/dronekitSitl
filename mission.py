#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string and drop coordinates
import argparse
parser = argparse.ArgumentParser(description='Drop mission - fly to coordinates and perform drop.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--drop-lat', type=float, required=False,
                    help="Latitude for drop location (e.g., 6.9735)")
parser.add_argument('--drop-lon', type=float, required=False,
                    help="Longitude for drop location (e.g., 3.6845)")
parser.add_argument('--drop-alt', type=float, default=15.0,
                    help="Altitude for drop location in meters (default: 15)")
parser.add_argument('--cruise-alt', type=float, default=10.0,
                    help="Cruise altitude in meters (default: 10)")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Get drop coordinates from command line or prompt user
drop_lat = args.drop_lat
drop_lon = args.drop_lon
drop_alt = args.drop_alt
cruise_alt = args.cruise_alt

if drop_lat is None or drop_lon is None:
    print("\n=== DROP MISSION COORDINATE INPUT ===")
    try:
        drop_lat_input = input("Enter drop latitude (e.g., 6.9735): ").strip()
        drop_lon_input = input("Enter drop longitude (e.g., 3.6845): ").strip()
        drop_alt_input = input("Enter drop altitude in meters (default 15): ").strip()
        cruise_alt_input = input("Enter cruise altitude in meters (default 10): ").strip()
        
        drop_lat = float(drop_lat_input)
        drop_lon = float(drop_lon_input)
        drop_alt = float(drop_alt_input) if drop_alt_input else 15.0
        cruise_alt = float(cruise_alt_input) if cruise_alt_input else 10.0
    except ValueError as e:
        print("Invalid coordinate input:", e)
        exit(1)

print("\n=== DROP MISSION PARAMETERS ===")
print("Drop Location: %.7f, %.7f" % (drop_lat, drop_lon))
print("Drop Altitude: %.1f m" % drop_alt)
print("Cruise Altitude: %.1f m" % cruise_alt)
print("================================\n")


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    # wait for mode to actually change
    while vehicle.mode.name != 'GUIDED':
        print(" Waiting for GUIDED mode...")
        time.sleep(1)
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """Returns the ground distance in metres between two LocationGlobalRelative objects."""
    lat1 = math.radians(aLocation1.lat)
    lon1 = math.radians(aLocation1.lon)
    lat2 = math.radians(aLocation2.lat)
    lon2 = math.radians(aLocation2.lon)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2.0)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    earth_radius = 6371000.0
    return earth_radius * c


def goto_and_wait(targetLocation, groundspeed=None, timeout=60, threshold=1.5):
    """Command vehicle.simple_goto and wait until arrival or timeout."""
    if groundspeed:
        vehicle.simple_goto(targetLocation, groundspeed=groundspeed)
    else:
        vehicle.simple_goto(targetLocation)

    start = time.time()
    while True:
        remaining = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print(" Distance to target: %.1f m" % (remaining,))
        if remaining <= threshold:
            print("Reached waypoint")
            break
        if time.time() - start > timeout:
            print("Timed out waiting for waypoint")
            break
        time.sleep(1)


def perform_drop(hover_time=2):
    """
    Perform the drop action at current location.
    Triggers servo on specified channel to release payload.
    
    Flight Controller Setup Required:
    - Set SERVOx_FUNCTION = 0 (Disabled) for your servo channel
    - Set SERVOx_MIN = 1100 and SERVOx_MAX = 1900
    - Common channels: AUX1=9, AUX2=10, AUX3=11, AUX4=12
    """
    from pymavlink import mavutil
    
    print("\n=== PERFORMING DROP ===")
    print("Hovering at drop location for %d seconds..." % hover_time)
    
    # Log current position
    current_loc = vehicle.location.global_relative_frame
    print("Current position: %.7f, %.7f, alt %.1f m" % (current_loc.lat, current_loc.lon, current_loc.alt))
    
    # Brief hover before drop
    time.sleep(hover_time)
    
    # === SERVO DROP CONFIGURATION ===
    # Modify these values for your setup:
    SERVO_CHANNEL = 9          # AUX1=9, AUX2=10, AUX3=11, AUX4=12
    SERVO_RELEASE_PWM = 1100   # PWM value to release (typically 1100-1200)
    SERVO_CLOSED_PWM = 1900    # PWM value for closed/hold position
    
    print("Triggering servo release on channel %d (PWM=%d)" % (SERVO_CHANNEL, SERVO_RELEASE_PWM))
    
    # Send servo command to release payload
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
        0,       # confirmation
        SERVO_CHANNEL,      # param1: servo number
        SERVO_RELEASE_PWM,  # param2: PWM value
        0, 0, 0, 0, 0)      # unused parameters
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    print("Servo triggered - payload released!")
    time.sleep(1)  # Wait for payload to clear
    
    # Return servo to closed position
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_CHANNEL,
        SERVO_CLOSED_PWM,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print("Servo returned to closed position")
    
    print("DROP COMPLETE!")
    print("========================\n")
    time.sleep(0.5)


arm_and_takeoff(cruise_alt)

# Speed configuration: increase these to make the drone faster
FAST_AIRSPEED = 15    # m/s (used by vehicle.airspeed)
FAST_GROUNDSPEED = 15 # m/s (used for simple_goto groundspeed)

print("Set default/target airspeed to %s" % FAST_AIRSPEED)
vehicle.airspeed = FAST_AIRSPEED

# Try to update the WPNAV_SPEED parameter (cm/s) if available
try:
    wp_speed_cm_s = int(FAST_GROUNDSPEED * 100)
    vehicle.parameters['WPNAV_SPEED'] = wp_speed_cm_s
    print("Set WPNAV_SPEED to %d cm/s" % wp_speed_cm_s)
except Exception as e:
    print("Could not set WPNAV_SPEED parameter:", e)

# Get home location for reference
home_lat = vehicle.location.global_relative_frame.lat
home_lon = vehicle.location.global_relative_frame.lon
print("Home coordinates: %.7f, %.7f" % (home_lat, home_lon))

# Create drop location waypoint
drop_location = LocationGlobalRelative(drop_lat, drop_lon, drop_alt)

print("\n=== STARTING DROP MISSION ===")
print("Flying to drop location: %.7f, %.7f, alt %.1f m" % (drop_lat, drop_lon, drop_alt))

# Navigate to drop location
goto_and_wait(drop_location, groundspeed=FAST_GROUNDSPEED, timeout=120, threshold=2.0)

# Perform the drop
perform_drop(hover_time=3)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")
while vehicle.mode.name != 'RTL':
    print(" Waiting for RTL mode to engage...")
    time.sleep(1)
time.sleep(5)


def wait_for_landing_and_disarm(timeout=300, land_alt_threshold=0.5):
    """Wait until the vehicle has landed (altitude <= threshold) and disarm motors.

    - timeout: max seconds to wait for landing
    - land_alt_threshold: altitude in metres considered landed
    Returns True if disarmed, False on timeout.
    """
    print("Waiting for vehicle to land (threshold %.2f m)" % land_alt_threshold)
    start = time.time()
    while time.time() - start < timeout:
        alt = None
        try:
            alt = vehicle.location.global_relative_frame.alt
        except Exception:
            pass
        if alt is not None:
            print(" Altitude: %.2f m" % (alt,))
            if alt <= land_alt_threshold:
                print("Landed (alt <= %.2f). Disarming motors..." % land_alt_threshold)
                try:
                    vehicle.armed = False
                except Exception as e:
                    print("Failed to set armed=False:", e)
                # wait until actually disarmed
                disarm_start = time.time()
                while vehicle.armed and time.time() - disarm_start < 30:
                    print(" Waiting for disarm...")
                    time.sleep(1)
                if not vehicle.armed:
                    print("Vehicle disarmed.")
                    return True
                else:
                    print("Vehicle still armed after attempt.")
                    return False
        else:
            print(" Altitude unknown, still waiting...")
        time.sleep(1)
    print("Timeout waiting for landing (%ds)" % timeout)
    # attempt to disarm anyway
    try:
        vehicle.armed = False
    except Exception as e:
        print("Failed to disarm on timeout:", e)
    return not vehicle.armed

# Wait for landing and disarm before closing
wait_for_landing_and_disarm(timeout=300, land_alt_threshold=0.5)

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()