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
            self.eventScanComplete = threading.Event()
            self.eventPlantComplete = threading.Event()
            self.eventDistanceThreadActive = threading.Event()
            self.eventPlantLocationReached = threading.Event()
            self.origin = np.array
            self.vision_tcp = None
            self.detect = 0
            self.eventObjectDetected = threading.Event()
            self.eventCircleStart = threading.Event()

        except dk.APIException:
            print("Timeout")

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        self.eventThreadActive.set()
        print("[INFO Drone] Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" [INFO Drone] Waiting for vehicle to initialise...")
            time.sleep(1)

        print("[INFO Drone] Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" [INFO Drone] Waiting for arming...")
            time.sleep(1)

        print("[INFO Drone] Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print("[INFO Drone] Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.97:
                print("[INFO Drone] Reached target altitude")
                break
            time.sleep(1)

        self.eventTakeOffComplete.set()
        self.eventThreadActive.clear()

    def get_current_location(self):
        current_location = self.vehicle.location.global_relative_frame
        # current_location = self.vehicle.location.global_relative_alt
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
        print("[INFO Drone] Flying towards point")
        self.vehicle.simple_goto(location)
        while True:
            lat = self.vehicle.location.global_relative_frame.lat
            long = self.vehicle.location.global_relative_frame.lon
            distance = distance_between(lat, long, location.lat, location.lon, self.origin)
            print("[INFO Drone] distance to point:", distance)
            if distance <= 1:
                self.eventLocationReached.set()
                self.eventThreadActive.clear()
                break

            time.sleep(3)

        return print("[INFO Drone] location reached")

    def set_plant_flag(self):
        self.eventThreadActive.set()
        for i in range(3):
            print("[INFO Drone] planting")
            time.sleep(1)
        print("[INFO Drone] planting complete")
        self.eventPlantComplete.set()
        self.eventThreadActive.clear()

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

    def circle_velocities(self, radius, duration):
        coord_north = []
        coord_east = []

        # create a set of 360 points east and north that describe a circle counter clockwise
        for i in range(360):
            rad = math.radians(i)
            coord_north.append(radius * math.sin(rad))
            coord_east.append(radius * math.cos(rad))

        # get the vectors which joint these points, counter clockwise
        vec_north = []
        vec_east = []
        for i in range(359):
            vec_north.append(coord_north[i+1] - coord_north[i])
            vec_east.append(coord_east[i+1] - coord_east[i])

        # the time for each vector
        time_per = duration / len(vec_north)

        # define the velocities north and east, the vectors divided by the time per
        vel_north = []
        vel_east = []

        for i in range(len(vec_north)):
            vel_north.append(vec_north[i]/time_per)
            vel_east.append(vec_east[i]/time_per)

        return vel_north, vel_east

    def set_roi(self, lat, lon, alt):
        # create the MAV_CMD_DO_SET_ROI command
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI,  # command
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            lat,
            lon,
            alt)
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

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

        if duration == int:
            # send command to vehicle on 1 Hz cycle
            for x in range(0, duration):
                self.vehicle.send_mavlink(msg)
                time.sleep(1)
        else:
            for x in range(0, 1):
                self.vehicle.send_mavlink(msg)
                time.sleep(duration)

    '''def the_only_real_scan_shady(self, duration, radius):
        self.eventThreadActive.set()
        # change this later so it uses the current way point target
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        wp = np.array([lat, lon, alt])
        self.manual_circle(wp, duration, radius)
        print('[INFO Drone] scan complete')
        self.eventScanComplete.set()
        self.eventThreadActive.clear()'''

    def circle(self, duration, radius):
        vel_north, vel_east = self.circle_velocities(radius, duration)  # get velocities
        time_per = duration / len(vel_north)  # get time per

        # move to outer edge of circle and yaw to centre
        # self.send_yaw(270, 90, 0)
        self.send_global_velocity(0, radius/2, 0, 2)
        self.eventCircleStart.set()
        for i in range(len(vel_north)):
            self.send_global_velocity(vel_north[i], vel_east[i], 0, time_per)

    def scan(self, location, duration, radius):
        lat = location.lat
        lon = location.lon
        alt = location.alt
        self.set_roi(lat, lon, alt)
        print('[INFO SCAN] >> ROI set')
        self.circle(duration, radius)
        self.vehicle.simple_goto(self.get_plant_location(lat, lon, alt))
        print('[INFO SCAN] >> scan complete')
        self.eventScanComplete.set()

    def handle_unity(self):
        tcp = TCP(5598, 'UNITY')  # create instance of tcp class
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            string = self.get_positional_data()
            tcp.send_message(string)

    def handle_vision(self):
        tcp = TCP(5311, 'VISION')
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            msg = tcp.receive_message()
            self.detect = int(msg)
            if self.detect == 1:
                print(self.detect)

    def scan_output(self, duration):
        self.eventCircleStart.wait()
        t_end = time.time() + duration
        frame_count = 0
        detect_count = 0
        while True:
            if time.time() > t_end:
                print(f'[INFO VISION] >> total frames = {frame_count}')
                percent = (detect_count / frame_count)*100
                print(f'[INFO VISION] >> detected frames = {detect_count}, {percent}')
                break
            if self.detect == 1:
                self.eventObjectDetected.set()
                detect_count += 1
            frame_count += 1
