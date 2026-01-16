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


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


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


arm_and_takeoff(10)

# Speed configuration: increase these to make the drone faster
FAST_AIRSPEED = 8    # m/s (used by vehicle.airspeed)
FAST_GROUNDSPEED = 8 # m/s (used for simple_goto groundspeed)

print("Set default/target airspeed to %s" % FAST_AIRSPEED)
vehicle.airspeed = FAST_AIRSPEED

# Try to update the WPNAV_SPEED parameter (cm/s) if available
try:
    wp_speed_cm_s = int(FAST_GROUNDSPEED * 100)
    vehicle.parameters['WPNAV_SPEED'] = wp_speed_cm_s
    print("Set WPNAV_SPEED to %d cm/s" % wp_speed_cm_s)
except Exception as e:
    print("Could not set WPNAV_SPEED parameter:", e)
print("Using home coordinates: 6.9732271, 3.6842822")

# Build a small square of realistic waypoints around the home position
home_lat = 6.9732271
home_lon = 3.6842822
alt = 15
delta = 0.00012  # ~13m
waypoints = [
    LocationGlobalRelative(home_lat + delta, home_lon, alt),
    LocationGlobalRelative(home_lat + delta, home_lon + delta, alt),
    LocationGlobalRelative(home_lat, home_lon + delta, alt),
    LocationGlobalRelative(home_lat - delta, home_lon + delta, alt),
]

for i, wp in enumerate(waypoints, start=1):
    print("Going to waypoint %d: %.7f, %.7f, alt %.1f" % (i, wp.lat, wp.lon, wp.alt))
    # wait up to 45s for each waypoint at faster groundspeed, threshold 2m
    goto_and_wait(wp, groundspeed=FAST_GROUNDSPEED, timeout=45, threshold=2.0)
    # small pause between waypoints
    time.sleep(1)

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