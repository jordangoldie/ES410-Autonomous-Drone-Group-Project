import dronekit as dk                # import drone kit library
from dronekit import connect         # import connect method from drone kit
from dronekit import VehicleMode     # import VehicleMode object from drone kit
from pymavlink import mavutil
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
            self.eventTakeOffComplete = threading.Event()
            self.eventMissionComplete = threading.Event()
            self.eventThreadActive = threading.Event()
            self.eventLocationReached = threading.Event()
            self.eventObjectDetected = threading.Event()
            self.eventScanComplete = threading.Event()
            self.eventPlantComplete = threading.Event()
            self.eventDistanceThreadActive = threading.Event()
            self.eventPlantLocationReached = threading.Event()
            self.origin = np.array
            self.vision_tcp = None

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
        while True:
            lat = self.vehicle.location.global_relative_frame.lat
            long = self.vehicle.location.global_relative_frame.lon
            distance = distance_between(lat, long, location.lat, location.lon, self.origin)
            print("distance to point:", distance)
            if distance <= 1:
                self.eventLocationReached.set()
                self.eventThreadActive.clear()
                break

            time.sleep(3)

        return print("location reached")

    def set_plant_flag(self):
        self.eventThreadActive.set()
        for i in range(3):
            print("planting")
            time.sleep(1)
        print("planting complete")
        self.eventPlantComplete.set()
        self.eventThreadActive.clear()

    def circle(self):
        self.eventThreadActive.set()
        # change this later so it uses the current waypoint target
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        wp = np.array([lat, lon, alt])
        self.get_circle_coords(lat, lon)
        self.manual_circle(wp)
        self.eventScanComplete.set()
        self.eventThreadActive.clear()

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
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
        for x in range(0, 1):
            self.vehicle.send_mavlink(msg)
            time.sleep(duration)

    def send_yaw(self, angle, speed, relative):
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            angle,  # param 1, yaw in degrees
            speed,  # param 2, yaw speed deg/s
            -1,  # param 3, direction -1 ccw, 1 cw
            relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def send_global_velocity2(self, velocity_x, velocity_y, velocity_z, duration):
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
        self.vehicle.mode = VehicleMode("RTL")
        self.eventThreadActive.set()

    def get_positional_data(self):
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        vec_local = get_vector(self.origin, lat, lon)

        roll = self.get_attitude().roll
        pitch = self.get_attitude().pitch
        yaw = self.get_attitude().yaw

        string = f'{vec_local[0]},{vec_local[1]},{alt},{roll},{pitch},{yaw}'

        return string

    # now that simple go to is not being used, you can change this as the vel should be the same each time
    def circle_velocities(self, lat, long, radius, duration):
        wp_pos = get_vector(self.origin, lat, long)

        circle_x = []
        circle_y = []

        for i in range(0, 360):
            rad = math.radians(i)
            circle_y.append(wp_pos[1] + radius * math.cos(rad))
            circle_x.append(wp_pos[0] + radius * math.sin(rad))

        vec_num = len(circle_x) - 1
        step_time = duration/vec_num

        vec_x = []
        vec_y = []
        vel_x = []
        vel_y = []

        for i in range(0, vec_num):
            vec_x.append(circle_x[i + 1] - circle_x[i])
            vec_y.append(circle_y[i + 1] - circle_y[i])
            vel_x.append(vec_x[i]/step_time)
            vel_y.append(vec_y[i]/step_time)

        return vel_x, vel_y

    def manual_circle(self, wp, duration, radius):
        vel_x, vel_y = self.circle_velocities(wp[0], wp[1], radius, duration)
        step_time = duration/len(vel_x)

        speed = 1.5
        self.send_yaw(270, 90, 0)
        self.send_global_velocity2(0, speed, 0, 2)

        for i in range(0, len(vel_x)):
            self.send_global_velocity(vel_x[i], vel_y[i], 0, step_time)
            self.send_yaw(3.5, 18, 1)

    def the_only_real_scan_shady(self, duration, radius):
        self.eventThreadActive.set()
        # change this later so it uses the current way point target
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        wp = np.array([lat, lon, alt])
        self.manual_circle(wp, duration, radius)
        print('scan complete')
        self.eventScanComplete.set()
        self.eventThreadActive.clear()

    def scan(self, detection):
        self.eventThreadActive.set()

        for i in range(5):
            time.sleep(1)
            print("scanning")
        print("scan complete")

        if detection == 1:
            self.eventObjectDetected.set()

        self.eventScanComplete.set()
        self.eventThreadActive.clear()

    def handle_unity(self):
        tcp = TCP(5598)  # create instance of tcp class
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            string = self.get_positional_data()
            tcp.send_message(string)


