import dronekit as dk                # import dronekit library
from dronekit import connect         # import connect method from dronekit
from dronekit import VehicleMode     # import VehicleMode object from dronekit
import time                          # import time library
import math                          # import math library
import threading

# Drone class
class Drone:
    def __init__(self, connection_str):
        try:
            self.vehicle = connect(connection_str)
            self.event_flag = 0
            self.eventTakeOffComplete = threading.Event()
            self.eventMissionComplete = threading.Event()
            self.eventThreadActive = threading.Event()
            self.eventLocationReached = threading.Event()
            self.eventObjectDetected = threading.Event()
            self.eventScanComplete = threading.Event()
            self.eventPlant = threading.Event()
            self.eventDistanceThreadActive = threading.Event()

        except dk.APIException:
            print("Timeout")

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        self.eventThreadActive.set()
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.97:
                print("Reached target altitude")
                break
            time.sleep(1)

        self.eventTakeOffComplete.set()
        self.eventThreadActive.clear()


    def distance_to_point_m(self, point):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        Modified from dronekit example documentation
        The final term deals with the earths curvature
        """
        dlat = point.lat - self.vehicle.location.global_relative_frame.lat
        dlong = point.lon - self.vehicle.location.global_relative_frame.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
        return distance

    def get_current_location(self):
        current_location = self.vehicle.location.global_relative_frame
        return current_location

    def get_velocity(self):
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

    def fly_to_point(self, location, airspeed):
        self.eventThreadActive.set()
        self.vehicle.airspeed = airspeed
        print("Flying towards point")
        self.vehicle.simple_goto(location)

    def fly_to_point2(self, location, airspeed):
        self.eventPlantLocationReached.wait()
        self.vehicle.airspeed = airspeed
        print("Flying towards point")
        self.vehicle.simple_goto(location)

    def check_distance(self, plant_location):
        dlat = plant_location.lat - self.vehicle.location.global_relative_frame.lat
        dlong = plant_location.lon - self.vehicle.location.global_relative_frame.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
        while True:
            time.sleep(1)
            dlat = plant_location.lat - self.vehicle.location.global_relative_frame.lat
            dlong = plant_location.lon - self.vehicle.location.global_relative_frame.lon
            distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
            print(distance)
            if distance <= 1:
                self.eventLocationReached.set()
                self.eventThreadActive.clear()
                break
            else:
                self.eventLocationReached.clear()
        return print("location reached")

    def plant_wait(self, plant_time):
        for i in range(plant_time):
            print("planting...")
            time.sleep(1)

    def set_plant_flag(self):
        self.eventThreadActive.set()
        self.eventPlant.set()
        self.eventThreadActive.clear()

    def circle(self):
        self.vehicle.mode = VehicleMode("CIRCLE")

    def event(self):
        flag = self.event_flag
        print("flag", flag)

        #if flag == 0:
        #    return
        #elif flag == 1:
        #    event = "take off complete"
        #elif flag == 2:
         #   event = "plant location reached"
        #else:
         #   print("flag not set")
        #return event

    def reset_event_flag(self):
        self.event_flag = 0

    def return_home(self):
        self.vehicle.mode = VehicleMode("RTL")



