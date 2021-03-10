import dronekit as dk                # import drone kit library
from dronekit import connect         # import connect method from drone kit
from dronekit import VehicleMode     # import VehicleMode object from drone kit
import time                          # import time library
import math                          # import math library
import threading
from GPS_integration import get_vector, distance_between
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
            self.eventStartTime = time.time()

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

    def get_airspeed(self): # pragma:  no cover
        airspeed = self.vehicle.airspeed
        return airspeed

    def get_ground_speed(self): # pragma:  no cover
        ground_speed = self.vehicle.groundspeed
        return ground_speed

    def get_attitude(self): # pragma:  no cover
        attitude = self.vehicle.attitude
        return attitude

    def get_plant_location(self, lat, lon, alt): # pragma:  no cover
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

    def descend(self, target_alt): # pragma no cover
        print(f'[INFO EVENT] >> Event: descend thread started, Timestamp:{datetime.now()}')
        self.eventLocationReached.wait()
        print(f'[INFO EVENT] >> Event: LocationReached.wait() passed, Timestamp:{datetime.now()}')
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print('[INFO DESCEND] >> Altitude: ', self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just above planting altitude.
            if self.vehicle.location.global_relative_frame.alt <= target_alt * 1.03:
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
        self.eventPlantComplete.wait()
        print(f'[INFO EVENT] >> Event: PlantComplete.wait() passed, Timestamp:{datetime.now()}')
        location = self.get_current_location()
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)
        self.vehicle.simple_goto(new_position)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Break and return from function just below flying altitude.
            if self.vehicle.location.global_relative_frame.alt >= target_alt * 0.97:
                print('[INFO ASCEND] >> Reached flying altitude')
                self.eventFlyingAltReached.set()
                print(f'[INFO EVENT] >> Event: FlyingAltReached.set(), Timestamp:{datetime.now()}')
                self.eventLocationReached.clear()
                print(f'[INFO EVENT] >> Event: LocationReached.clear(), Timestamp:{datetime.now()}')
                self.eventPlantAltReached.clear()
                print(f'[INFO EVENT] >> Event: PlantAltReached.clear(), Timestamp:{datetime.now()}')
                self.eventPlantComplete.clear()
                print(f'[INFO EVENT] >> Event: PlantComplete.clear(), Timestamp:{datetime.now()}')
                self.eventMissionComplete.set()
                print(f'[INFO EVENT] >> Event: MissionComplete.set(), Timestamp:{datetime.now()}')
                self.waypoint_count += 1
                break
            time.sleep(1)

    def return_home(self): # pragma:  no cover
        print(f'[INFO EVENT] >> Event: return_home thread started, Timestamp:{datetime.now()}')
        self.eventMissionComplete.wait()
        print(f'[INFO EVENT] >> Event: MissionComplete.wait() passed, Timestamp:{datetime.now()}')
        self.vehicle.mode = VehicleMode("RTL")

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration): # pragma:  no cover
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

    def get_positional_data(self): # pragma:  no cover
        lat = self.get_current_location().lat
        lon = self.get_current_location().lonS
        alt = self.get_current_location().alt
        vector_fn = get_vector(self.origin, lat, lon)

        roll = self.get_attitude().roll
        pitch = self.get_attitude().pitch
        yaw = self.get_attitude().yaw

        string = f'{vector_fn[0]},{vector_fn[1]},{alt},{roll},{pitch},{yaw}'

        return string

    def circle_velocities(self, radius, duration): # pragma:  no cover
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

    def set_roi(self, lat, lon, alt): # pragma:  no cover
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

    def send_yaw(self, angle, speed, relative): # pragma:  no cover
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

    def circle(self, duration, radius): # pragma:  no cover
        vel_north, vel_east = self.circle_velocities(radius, duration)  # get velocities
        time_per = duration / len(vel_north)  # get time per

        # move to outer edge of circle and yaw to centre
        # self.send_yaw(270, 90, 0)
        self.send_global_velocity(0, radius, 0, 2)
        for i in range(len(vel_north)):
            self.send_global_velocity(vel_north[i], vel_east[i], 0, time_per)

    def scan(self, location, duration, radius): # pragma:  no cover
        self.eventPlantAltReached.wait()
        lat = location.lat
        lon = location.lon
        alt = location.alt
        self.set_roi(lat, lon, alt)
        print('[INFO SCAN] >> ROI set')
        self.circle(duration, radius)
        self.vehicle.simple_goto(self.get_plant_location(lat, lon, alt))
        print('[INFO SCAN] >> scan complete')
        self.eventScanComplete.set()

    def handle_unity(self): # pragma:  no cover
        tcp = TCP(5598, 'UNITY')  # create instance of tcp class
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            string = self.get_positional_data()
            tcp.send_message(string)

    def handle_vision(self): # pragma:  no cover
        tcp = TCP(5311, 'VISION')
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            msg = tcp.receive_message()
            self.detect = int(msg)
            if self.detect == 1:
                print(self.detect)

    def scan_output(self, duration): # pragma:  no cover
        self.eventPlantAltReached.wait()
        t_end = time.time() + duration
        while True:
            if time.time() > t_end:
                break
            if self.detect == 1:
                self.eventObjectDetected.set()

    def scan_output_test(self, duration): # pragma:  no cover
        self.eventPlantAltReached.wait()
        t_end = time.time() + duration
        while True:
            if time.time() > t_end:
                break
            if self.detect == 1:
                self.eventObjectDetected.set()

