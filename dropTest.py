#!/usr/bin/env python
"""dropTest.py

Fly to a drop waypoint, announce "droping package" when arrived,
then RTL and wait for landing and disarm.

Usage: python dropTest.py --connect <connection_string>
"""
from __future__ import print_function
import time
import math
import argparse
try:
    import pyttsx3
    _have_tts = True
except Exception:
    _have_tts = False

from dronekit import connect, VehicleMode, LocationGlobalRelative


def announce(text):
    """Try to say text with TTS, fallback to print."""
    if _have_tts:
        try:
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
            return
        except Exception:
            pass
    print(text)


def get_distance_metres(aLocation1, aLocation2):
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


def goto_and_wait(vehicle, targetLocation, groundspeed=None, timeout=90, threshold=2.0):
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
            return True
        if time.time() - start > timeout:
            print("Timed out waiting for waypoint")
            return False
        time.sleep(1)


def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != 'GUIDED':
        print(" Waiting for GUIDED mode...")
        time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(" Altitude: ", alt)
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def wait_for_landing_and_disarm(vehicle, timeout=300, land_alt_threshold=0.5):
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
    try:
        vehicle.armed = False
    except Exception as e:
        print("Failed to disarm on timeout:", e)
    return not vehicle.armed


def main():
    parser = argparse.ArgumentParser(description='Drop test')
    parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL started.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

    try:
        arm_and_takeoff(vehicle, 12)

        FAST_AIRSPEED = 6
        FAST_GROUNDSPEED = 6
        vehicle.airspeed = FAST_AIRSPEED
        try:
            vehicle.parameters['WPNAV_SPEED'] = int(FAST_GROUNDSPEED * 100)
        except Exception:
            pass

        # Drop waypoint requested by user
        drop_lat = 6.9778007
        drop_lon = 3.6628544
        drop_alt = 10
        drop_wp = LocationGlobalRelative(drop_lat, drop_lon, drop_alt)

        print("Going to drop waypoint: %.7f, %.7f, alt %.1f" % (drop_lat, drop_lon, drop_alt))
        arrived = goto_and_wait(vehicle, drop_wp, groundspeed=FAST_GROUNDSPEED, timeout=120, threshold=3.0)
        if arrived:
            announce("droping package")
            # simulate drop delay
            time.sleep(2)
        else:
            print("Did not reach drop waypoint within timeout; proceeding to RTL")

        print("Returning to Launch (RTL)")
        vehicle.mode = VehicleMode('RTL')
        while vehicle.mode.name != 'RTL':
            print(" Waiting for RTL mode to engage...")
            time.sleep(1)

        # wait for landing and disarm
        wait_for_landing_and_disarm(vehicle, timeout=300, land_alt_threshold=0.5)

    finally:
        print("Close vehicle object")
        vehicle.close()
        if sitl:
            sitl.stop()


if __name__ == '__main__':
    main()
