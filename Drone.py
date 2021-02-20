import dronekit as dk                # import drone kit library
from dronekit import connect         # import connect method from drone kit
from dronekit import VehicleMode     # import VehicleMode object from drone kit
from pymavlink import mavutil
import time                          # import time library
import math                          # import math library
import threading
from GPS2 import get_vector, get_gps
import numpy as np


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
            self.eventPlantLocationReached = threading.Event()
            self.origin = np.array

        except dk.APIException:
            print("Timeout")

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        self.eventTakeOffComplete.set()
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

        self.eventThreadActive.clear()

    def distance_to_point_m(self, point):
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

    def check_distance(self, location, plant_indicator):
        '''dlat = plant_location.lat - self.vehicle.location.global_relative_frame.lat
        dlong = plant_location.lon - self.vehicle.location.global_relative_frame.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5'''
        while True:
            time.sleep(1)

            '''Returns the ground distance in metres between two LocationGlobal objects.

               This method is an approximation, and will not be accurate over large distances and close to the 
               earth's poles. It comes from the ArduPilot test code: 
               https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py'''
            dlat = location.lat - self.vehicle.location.global_relative_frame.lat
            dlong = location.lon - self.vehicle.location.global_relative_frame.lon
            distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
            print(distance)
            if distance <= 1:
                if plant_indicator == 1:
                    self.eventPlantLocationReached.set()
                    self.eventThreadActive.clear()
                    self.eventDistanceThreadActive.clear()
                else:
                    self.eventLocationReached.set()
                    self.eventThreadActive.clear()
                    self.eventDistanceThreadActive.clear()
                break
            else:
                self.eventLocationReached.clear()
            time.sleep(2)

        return print("location reached")

    def plant_wait(self, plant_time):
        for i in range(plant_time):
            print("planting...")
            time.sleep(1)

    def set_plant_flag(self):
        for i in range(3):
            print("planting")
            time.sleep(1)
        print("planting complete")
        self.eventThreadActive.set()
        self.eventPlant.set()
        self.eventThreadActive.clear()
        self.eventScanComplete.clear()

    def circle(self):
        self.eventThreadActive.set()
        # change this later so it uses the current waypoint target
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        wp = np.array([lat, lon, alt])
        self.get_circle_coords(lat, lon, self.origin)
        self.manual_circle(wp, self.origin)
        self.eventScanComplete.set()
        self.eventThreadActive.clear()

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def event(self):
        flag = self.event_flag
        print("flag", flag)

    def reset_event_flag(self):
        self.event_flag = 0

    def return_home(self):
        self.vehicle.mode = VehicleMode("RTL")
        self.eventThreadActive.set()

    def get_positional_data(self, origin):
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        vector_fn = get_vector(origin, lat, lon)

        roll = self.get_attitude().roll
        pitch = self.get_attitude().pitch
        yaw = self.get_attitude().yaw

        string = f'{vector_fn[0]},{vector_fn[1]},{alt},{roll},{pitch},{yaw}'

        return string

    def get_circle_coords(self, lat, long, origin):
        radius = 2
        wp_pos = get_vector(origin, lat, long)

        circle_x = []
        circle_y = []
        circle_lats = []
        circle_longs = []

        for i in range(0, 360):
            rad = int(i) * (math.pi/180)
            circle_x.append(wp_pos[0] + radius * math.cos(rad))
            circle_y.append(wp_pos[1] + radius * math.sin(rad))
            circle_point = np.array([circle_x[i], circle_y[i], wp_pos[2], 1])
            circle_point = np.transpose(circle_point)
            lat, lon, long2 = get_gps(origin, circle_point)
            circle_lats.append(lat)
            circle_longs.append(long2)

        return circle_lats, circle_longs

    def manual_circle(self, wp, origin):
        lats, longs = self.get_circle_coords(wp[0], wp[1], origin)

        circle_p = self.get_plant_location(lats[0], longs[0], wp[2])
        self.vehicle.simple_goto(circle_p)
        time.sleep(3)

        for i in range(0, len(lats)):
            circle_p = self.get_plant_location(lats[i], longs[i], wp[2])
            self.fly_to_point(circle_p, 5)
            time.sleep(0.5)

'''def orbit(self):
        """
                Move vehicle in direction based on specified velocity vectors.
                """
        msg = self.vehicle.message_factory.mav_cmd_do_orbit_encode(
            ,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            0b00000111,  # type_mask (only speeds enabled)
            {1, 0, 0, 0},
            0, 0, 0,
            0.5)

        self.vehicle.send_mavlink(msg)'''


