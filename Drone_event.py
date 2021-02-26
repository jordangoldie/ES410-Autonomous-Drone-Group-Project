import dronekit as dk                # import drone kit library
from dronekit import connect         # import connect method from drone kit
from dronekit import VehicleMode     # import VehicleMode object from drone kit
from pymavlink import mavutil
import serial
import time                          # import time library
import math                          # import math library
import threading
from GPS import get_vector, get_gps, distance_between
import numpy as np
from TCP import TCP

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

        except dk.APIException:
            print("Timeout")

    def arm_and_takeoff(self, target_alt):
        """
        Arms vehicle and fly to target altitude.
        """
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
        self.vehicle.simple_takeoff(target_alt)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= target_alt * 0.97:
                print("Reached target altitude")
                break
            time.sleep(1)

        self.eventTakeOffComplete.set()
        self.eventFlyingAltReached.set()

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

    def adjust_altitude(self, target_alt):
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)
        self.vehicle.simple_goto(new_position)

    def check_altitude(self, target_alt):
        current_alt = self.get_current_location().alt
        diff = abs(target_alt - current_alt)
        return diff

    def fly_to_point(self, location, airspeed, plant_flag):
        self.eventTakeOffComplete.wait()
        self.eventFlyingAltReached.wait()
        self.vehicle.airspeed = airspeed
        print("Flying towards point")
        self.vehicle.simple_goto(location)
        while True:
            lat = self.vehicle.location.global_relative_frame.lat
            long = self.vehicle.location.global_relative_frame.lon
            distance = distance_between(lat, long, location.lat, location.lon, self.origin)
            print("distance to point:", distance)
            if distance <= 1:
                self.eventLocationReached.set()
                if plant_flag == 0:
                    self.eventLocationReached.clear()
                    self.eventThreadSeqActive.clear()
                break

            time.sleep(3)

        return print("location reached")

    def descend_to_plant_alt(self, plant_alt):
        self.eventLocationReached.wait()
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, plant_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just above planting altitude.
            if self.vehicle.location.global_relative_frame.alt <= plant_alt * 1.03:
                print("Reached planting altitude")
                self.eventPlantAltReached.set()
                self.eventFlyingAltReached.clear()
                break
            time.sleep(1)

    def scan(self, detection):
        self.eventPlantAltReached.wait()
        for i in range(5):
            time.sleep(1)
            print("scanning")
        print("scan complete")

        if detection == 1:
            self.eventObjectDetected.set()

        self.eventScanComplete.set()

    def set_plant_flag(self):
        self.eventScanComplete.wait()
        # send flag
        time.sleep(1)
        while True:
            load = input("Load: ")
            if load == "load_success":
                time.sleep(1)
                dispense = input("Dispense: ")
                print(dispense)
                break
            elif load == "Gate_Not_Positioned":
                time.sleep(1)
                print(load)
                break
        self.eventPlantComplete.set()

    def plant(self, flag):
        self.eventScanComplete.wait()
        for i in range(3):
            print("planting")
            time.sleep(1)
        print("planting complete")
        self.eventPlantComplete.set()

    def ascend_to_flying_alt(self, flying_alt):
        self.eventPlantComplete.wait()
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, flying_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below flying altitude.
            if self.vehicle.location.global_relative_frame.alt >= flying_alt * 0.97:
                print("Reached planting altitude")
                self.eventFlyingAltReached.set()
                self.eventPlantAltReached.clear()
                self.eventLocationReached.clear()
                self.eventPlantAltReached.clear()
                self.eventScanComplete.clear()
                self.eventPlantComplete.clear()
                self.eventThreadSeqActive.clear()
                self.waypoint_count += 1
                break
            time.sleep(1)

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

    def return_home(self):
        # self.eventMissionComplete.wait()
        self.vehicle.mode = VehicleMode("RTL")

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
            rad = math.radians(i)
            circle_y.append(wp_pos[1] + radius * math.cos(rad))
            circle_x.append(wp_pos[0] + radius * math.sin(rad))
            circle_point = np.array([circle_x[i], circle_y[i], wp_pos[2], 1])
            circle_point = np.transpose(circle_point)
            lat, lon, long2 = get_gps(origin, circle_point)
            circle_lats.append(lat)
            circle_longs.append(long2)

        return circle_x, circle_y

    def circle_velocities(self, circle_x, circle_y, duration):
        vec_num = len(circle_x) - 1
        step_time = duration/vec_num

        vec_x = []
        vec_y = []
        vel_x = []
        vel_y = []

        for i in range(0, vec_num):
            vec_x.append(circle_x[i+1] - circle_x[i])
            vec_y.append(circle_y[i + 1] - circle_y[i])
            vel_x.append(vec_x[i]/step_time)
            vel_y.append(vec_y[i]/step_time)

        return vel_x, vel_y

    def manual_circle(self, wp, origin, duration):
        circle_x, circle_y = self.get_circle_coords(wp[0], wp[1], origin)
        vel_x, vel_y = self.circle_velocities(circle_x, circle_y, 20)
        step_time = duration / len(vel_x)

        '''first_point = self.get_plant_location(lats[0], longs[0], wp[2])
        self.vehicle.simple_goto(first_point)
        time.sleep(1)'''

        for i in range(0, len(vel_x)):
            self.send_global_velocity(vel_x[i], vel_y[i], 0, step_time)

    def handle_unity(self):
        tcp = TCP(5598)  # create instance of tcp class
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            string = self.get_positional_data(self.origin)
            tcp.send_message(string)
            # speedx = self.get_ground_speed()
            # string = f'{speedx}, {speedy}, ...'
            # tcp.send_message(string)