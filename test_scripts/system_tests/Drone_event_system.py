import dronekit as dk                # import drone kit library
from dronekit import connect         # import connect method from drone kit
from dronekit import VehicleMode     # import VehicleMode object from drone kit
import time                          # import time library
import threading
from GPS_system import distance_between
import numpy as np
from datetime import datetime


# Drone class
class Drone:
    def __init__(self, connection_str):
        try:
            self.vehicle = connect(connection_str)
            self.event_flag = 0
            self.eventThreadSeqActive = threading.Event()
            self.eventMissionComplete = threading.Event()
            self.eventTakeOffComplete = threading.Event()
            self.eventLocationReached = threading.Event()
            self.eventPlantAltReached = threading.Event()
            self.eventScanComplete = threading.Event()
            self.eventObjectDetected = threading.Event()
            self.eventPlantComplete = threading.Event()
            self.eventFlyingAltReached = threading.Event()
            self.waypoint_count = 0
            self.origin = np.array
            self.vision_tcp = None
            self.detect = 0

        except dk.APIException: # pragma:  no cover
            print("Timeout")

    def arm_and_takeoff(self, target_alt):
        print(f'[INFO EVENT] >> Event: arm_and_takeoff thread started, Timestamp:{datetime.now()}')
        if target_alt <= 1:
            target_alt = 1
        elif target_alt >= 50:
            target_alt = 50
        """
        Arms vehicle and fly to target altitude.
        """
        print('[INFO ARM + TAKEOFF] >> Basic pre-arm checks')
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print('[INFO ARM + TAKEOFF] >> Waiting for vehicle to initialise...')
            time.sleep(1)

        print('[INFO ARM + TAKEOFF] >> Arming motors')
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print('[INFO ARM + TAKEOFF] >> Waiting for arming...')
            time.sleep(1)

        print('[INFO ARM + TAKEOFF] >> Taking off!')
        self.vehicle.simple_takeoff(target_alt)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print('[INFO ARM + TAKEOFF] >> Reached Altitude: ', self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= 0.98 * target_alt:
                print('[INFO ARM + TAKEOFF] >> Reached target altitude')
                break
            time.sleep(1)

        self.eventTakeOffComplete.set()
        print(f'[INFO EVENT] >>  Event: TakeOffComplete.set(), Timestamp:{datetime.now()}')
        self.eventFlyingAltReached.set()
        print(f'[INFO EVENT] >>  Event: FlyingAltReached.set(), Timestamp:{datetime.now()}')

    def get_current_location(self): # pragma:  no cover
        current_location = self.vehicle.location.global_relative_frame
        return current_location

    def get_velocity(self): # pragma:  no cover
        velocity = self.vehicle.velocity
        return velocity

    def get_airspeed(self):
        airspeed = self.vehicle.airspeed
        return airspeed

    def get_ground_speed(self):
        ground_speed = self.vehicle.groundspeed
        return ground_speed

    def get_attitude(self):
        attitude = self.vehicle.attitude
        return attitude

    def get_plant_location(self, lat, lon, alt):
        plant_location = dk.LocationGlobalRelative(lat, lon, alt)
        return plant_location

    def fly_to_point(self, location, airspeed, plant_flag):
        print(f'[INFO EVENT] >> Event: fly_to_point thread started, Timestamp:{datetime.now()}')
        self.eventTakeOffComplete.wait()
        print(f'[INFO EVENT] >> Event: TakeOffComplete.wait() passed, Timestamp:{datetime.now()}')
        self.eventFlyingAltReached.wait()
        print(f'[INFO EVENT] >> Event: FlyingAltReached.wait() passed, Timestamp:{datetime.now()}')
        self.vehicle.airspeed = airspeed
        print('[INFO FLY] >> Flying towards point')
        self.vehicle.simple_goto(location)
        while True:
            lat = self.vehicle.location.global_relative_frame.lat
            long = self.vehicle.location.global_relative_frame.lon
            distance = distance_between(lat, long, location.lat, location.lon, self.origin)
            print('[INFO FLY] >> distance to point:', distance)
            if distance <= 1:
                if plant_flag == 1:
                    self.eventLocationReached.set()
                    print(f'[INFO EVENT] >> Event: LocationReached.set(), Timestamp:{datetime.now()}')
                elif plant_flag == 0:
                    self.eventThreadSeqActive.clear()
                    print(f'[INFO EVENT] >> Event: ThreadSeqActive.clear(), Timestamp:{datetime.now()}')
                    self.waypoint_count += 1
                break

            time.sleep(2)

        return print("location reached")

    def descend(self, target_alt):
        print(f'[INFO EVENT] >> Event: descend thread started, Timestamp:{datetime.now()}')
        if target_alt <= 1:
            target_alt = 1
        elif target_alt >= 50:
            target_alt = 50

        self.eventLocationReached.wait()
        print(f'[INFO EVENT] >> Event: LocationReached.wait() passed, Timestamp:{datetime.now()}')
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print('[INFO DESCEND] >> Altitude: ', self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just above planting altitude.
            if self.vehicle.location.global_relative_frame.alt <= target_alt * 1.01:
                print('[INFO DESCEND] >> Reached planting altitude')
                self.eventPlantAltReached.set()
                print(f'[INFO EVENT] >> Event: PlantAltReached.set(), Timestamp:{datetime.now()}')
                self.eventFlyingAltReached.clear()
                print(f'[INFO EVENT] >> Event: FlyingAltReached.clear(), Timestamp:{datetime.now()}')
                break
            time.sleep(1)

    def plant(self): # pragma:  no cover
        print(f'[INFO EVENT] >> Event: plant thread started, Timestamp:{datetime.now()}')
        self.eventPlantAltReached.wait()
        print(f'[INFO EVENT] >> Event: PlantAltReached.wait() passed, Timestamp:{datetime.now()}')
        for i in range(3):
            print('[INFO PLANT] >> Dispensing')
            time.sleep(1)
            print('[INFO PLANT] >> Planting complete')
        self.eventPlantComplete.set()
        print(f'[INFO EVENT] >> Event: PlantComplete.set(), Timestamp:{datetime.now()}')

    def ascend(self, target_alt): # pragma:  no cover
        print(f'[INFO EVENT] >> Event: ascend thread started, Timestamp:{datetime.now()}')
        if target_alt <= 1:
            target_alt = 1
        elif target_alt >= 50:
            target_alt = 50

        self.eventPlantComplete.wait()
        print(f'[INFO EVENT] >> Event: PlantComplete.wait() passed, Timestamp:{datetime.now()}')
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below flying altitude.
            if self.vehicle.location.global_relative_frame.alt >= target_alt * 0.98:
                print('[INFO ASCEND] >> Reached flying altitude')
                self.eventFlyingAltReached.set()
                print(f'[INFO EVENT] >> Event: FlyingAltReached.set(), Timestamp:{datetime.now()}')
                self.eventLocationReached.clear()
                print(f'[INFO EVENT] >> Event: LocationReached.clear(), Timestamp:{datetime.now()}')
                self.eventPlantAltReached.clear()
                print(f'[INFO EVENT] >> Event: PlantAltReached.clear(), Timestamp:{datetime.now()}')
                self.eventPlantComplete.clear()
                print(f'[INFO EVENT] >> Event: PlantComplete.clear(), Timestamp:{datetime.now()}')
                self.eventThreadSeqActive.clear()
                print(f'[INFO EVENT] >> Event: ThreadSeqActive.clear(), Timestamp:{datetime.now()}')
                self.waypoint_count += 1
                break
            time.sleep(1)

    def return_home(self): # pragma:  no cover
        print(f'[INFO EVENT] >> Event: return_home thread started, Timestamp:{datetime.now()}')
        self.eventMissionComplete.wait()
        print(f'[INFO EVENT] >> Event: MissionComplete.wait() passed, Timestamp:{datetime.now()}')
        self.vehicle.mode = VehicleMode("RTL")


