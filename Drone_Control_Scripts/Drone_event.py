# import statements
import dronekit as dk                
from dronekit import connect         
from dronekit import VehicleMode     
from pymavlink import mavutil
import serial
import time                          
import math                          
import threading
from GPS import get_vector, get_gps, distance_between, set_origin
import numpy as np
from TCP import TcpServer


# Drone class
class Drone:
    # initialisation of relevant objects and variables
    def __init__(self, connection_str):
        try:
            # connect to SITL vehicle
            self.vehicle = connect(connection_str)                   
            
            # create events corresponding to drone control actions
            self.eventThreadSeqActive = threading.Event()
            self.eventMissionComplete = threading.Event()
            self.eventTakeOffComplete = threading.Event()
            self.eventLocationReached = threading.Event()
            self.eventPlantAltReached = threading.Event()
            self.eventScanComplete = threading.Event()
            self.eventObjectDetected = threading.Event()
            self.eventPlantComplete = threading.Event()
            self.eventFlyingAltReached = threading.Event()
            self.eventSendScanIndicator = threading.Event()           
            self.eventCircleStarted = threading.Event()          
            
            self.waypoint_count = 0                          # create waypoint counter     
            self.origin = set_origin(52.38255, -1.56156)     # set drone's origin
            self.detect = 0                                  # create person detection variable
            
        # print timeout if connection failure
        except dk.APIException:
            print("Timeout")

    # arm drone and take off to target altitude
    def arm_and_takeoff(self, target_alt):
        # limit target altitude to within 1 and 50 m
        if target_alt <= 1:
            target_alt = 1
        elif target_alt >= 50:
            target_alt = 50
        print('[INFO ARM] >> Basic pre-arm checks')                         
        
        # block arm attempt until autopilot is ready
        while not self.vehicle.is_armable:
            print('[INFO ARM] >> Waiting for vehicle to initialise...')     
            time.sleep(1)

        print('[INFO ARM] >> Arming motors')
        self.vehicle.mode = VehicleMode("GUIDED")     # set flight mode to guided     
        self.vehicle.armed = True                     # arm drone

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print('[INFO ARM] >> Waiting for arming...')
            time.sleep(1)

        print('[INFO TAKEOFF] >> Taking off!')
        self.vehicle.simple_takeoff(target_alt)     # Take off to target altitude

        # Wait until the drone gets within range of target altitude before next drone control command is executed
        while True:
            print("[TAKEOFF] >> Altitude: ", self.vehicle.location.global_relative_frame.alt)          
            if self.vehicle.location.global_relative_frame.alt >= target_alt * 0.98:              # Break from function if within 2 % of target altitude.
                print('[INFO TAKEOFF] >> Reached target altitude')
                break
            time.sleep(1)
       
        self.eventTakeOffComplete.set()      # set take off complete flag
        self.eventFlyingAltReached.set()     # set flying altitude reached flag

    # return drone current location
    def get_current_location(self):
        current_location = self.vehicle.location.global_relative_frame
        return current_location
    
    # return drone velocity
    def get_velocity(self):
        velocity = self.vehicle.velocity
        return velocity
    
    # return drone airspeed
    def get_airspeed(self):
        airspeed = self.vehicle.airspeed
        return airspeed

    # return drone groundspeed
    def get_ground_speed(self):
        ground_speed = self.vehicle.groundspeed
        return ground_speed

    # return drone atttide
    def get_attitude(self):
        attitude = self.vehicle.attitude
        return attitude

    # convert latitude, longitude and altitude into location object
    def get_plant_location(self, lat, lon, alt):
        plant_location = dk.LocationGlobalRelative(lat, lon, alt)
        return plant_location

    # command drone to fly to given location given airspeed, plant flag to specify if location requires planting
    def fly_to_point(self, location, airspeed, plant_flag): 
        
        # block fly to point command until take off is complete and flying altitude is reached
        self.eventTakeOffComplete.wait()
        self.eventFlyingAltReached.wait()
       
        self.vehicle.airspeed = airspeed                # set drone airspeed
        self.vehicle.simple_goto(location)              # command drone to fly to target location
        print('[INFO FLY] >> Flying towards point')     
        
        # Wait until drone gets within range of target location before next drone control command is executed
        while True:
            lat = self.vehicle.location.global_relative_frame.lat                               # return current drone latitude
            long = self.vehicle.location.global_relative_frame.lon                              # return current drone longitude
            distance = distance_between(lat, long, location.lat, location.lon, self.origin)     # return distance between drone position and target location
            print('[INFO FLY] >> distance to point:', distance)
            
            if distance <= 1:                             # break from function if within 1 m of target location                                      
                if plant_flag == 1:                       # if planting location set location reached flag
                    self.eventLocationReached.set()                                                
                if plant_flag == 0:                       # if not planting location reset thread sequence active flag                 
                    self.eventThreadSeqActive.clear()
                    self.waypoint_count += 1
                break

            time.sleep(3)     # pause between distance checks
            
        return print("location reached")

    # command drone to descend to target altitude
    def descend(self, target_alt):
        # limit target altitude to within 1 and 50 m
        if target_alt <= 1:
            target_alt = 1
        elif target_alt >= 50:
            target_alt = 50
            
        # block descend command until target location reached
        self.eventLocationReached.wait()     
        
        location = self.get_current_location()                                               # return drone current location 
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)     # define new target location with same altitude
        self.vehicle.simple_goto(new_position)                                               # command drone to travel to new location
        
        # wait until drone has descended to within range of target altitude before next command is executed
        while True:
            print('[INFO DESCEND] >> Altitude: ', self.vehicle.location.global_relative_frame.alt)       
            
            if self.vehicle.location.global_relative_frame.alt <= target_alt * 1.01:     # break from function if within 1 % of target altitude
                print('[INFO DESCEND] >> Reached planting altitude')
                self.eventPlantAltReached.set()                                          # set planting altitude reached flag
                self.eventFlyingAltReached.clear()                                       # reset flying altitude reached flag
                break
                
            time.sleep(1)     # pause between altitude checks

    def plant(self):
        
        # block planting until scan is complete
        self.eventScanComplete.wait()     
        
        # if object (person) detected abort planting, otherwise plant 
        if self.eventObjectDetected.is_set():
            print('[INFO PLANT] >> Planting aborted')
            print(f'[DEBUG PLANT] {self.eventObjectDetected.is_set()}')     #
        else:
            for i in range(3):
                print('[INFO PLANT] >> Dispensing')
                print(f'[DEBUG PLANT] {self.eventObjectDetected.is_set()}')
                time.sleep(1)
            print('[INFO PLANT] >> Planting complete')
            
        # set plant complete flag
        self.eventPlantComplete.set()     

    # command drone to ascend to target altitude    
    def ascend(self, target_alt):
        
        
        # block ascend command until planting is complete
        self.eventPlantComplete.wait()     
        
        location = self.get_current_location()                                               # return drone current location 
        new_position = dk.LocationGlobalRelative(location.lat, location.lon, target_alt)     # define new target location with same altitude
        self.vehicle.simple_goto(new_position)                                               # command drone to travel to new location
        
        # wait until drone has ascended to within range of target altitude before next command is executed 
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= target_alt * 0.98:     # break from function if within 2 % of target altitude
                print('[INFO ASCEND] >> Reached planting altitude')
                
                # reset event flag configuration
                self.eventFlyingAltReached.set()
                self.eventPlantAltReached.clear()
                self.eventLocationReached.clear()
                self.eventPlantAltReached.clear()
                self.eventScanComplete.clear()
                self.eventPlantComplete.clear()
                self.eventThreadSeqActive.clear()
                self.eventObjectDetected.clear()
                self.waypoint_count += 1
                break
                
            time.sleep(1)     # pause between altitude checks

    # command drone to return to home location and land
    def return_home(self):
        # self.eventMissionComplete.wait() ###########################################################################
        self.vehicle.mode = VehicleMode("RTL")     # set vehicle mode to RTL, causing drone to return home and land
        
    # send XYZ velocity commands to the drone to be excecuted for given duration
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

    def get_positional_data(self):
        lat = self.get_current_location().lat
        lon = self.get_current_location().lon
        alt = self.get_current_location().alt
        vector_fn = get_vector(self.origin, lat, lon)

        roll = self.get_attitude().roll
        pitch = self.get_attitude().pitch
        yaw = self.get_attitude().yaw

        if self.eventSendScanIndicator.is_set():
            scan = 1
        else:
            scan = 0

        string = f'{vector_fn[0]},{vector_fn[1]},{alt},{roll},{pitch},{yaw}, {scan}'

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

    def circle(self, duration, radius):
        vel_north, vel_east = self.circle_velocities(radius, duration)  # get velocities
        time_per = duration / len(vel_north)  # get time per

        # move to outer edge of circle and yaw to centre
        # self.send_yaw(270, 90, 0)
        self.send_global_velocity(0, radius/2, 0, 2)
        for i in range(len(vel_north)):
            self.send_global_velocity(vel_north[i], vel_east[i], 0, time_per)

    def scan(self, location, duration, radius):
        self.eventPlantAltReached.wait()
        lat = location.lat
        lon = location.lon
        alt = location.alt
        self.set_roi(lat, lon, alt)
        print('[INFO SCAN] >> ROI set')
        self.circle(duration, radius)
        self.vehicle.simple_goto(location)
        time.sleep(3)
        print('[INFO SCAN] >> scan complete')
        self.eventScanComplete.set()

    def handle_unity(self):
        tcp = TcpServer(5598, 'UNITY')  # create instance of tcp class
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            string = self.get_positional_data()
            tcp.send_message(string)

    def handle_vision(self):
        tcp = TcpServer(5311, 'VISION')
        tcp.bind_server_socket()
        tcp.listen_for_tcp()

        while True:
            msg = tcp.receive_message()
            self.detect = int(msg)
            print(self.detect)

    def scan_output(self, duration):
        self.eventPlantAltReached.wait()
        self.eventSendScanIndicator.set()
        print(f'[DEBUG scan output] Scan indicator: {self.eventSendScanIndicator.is_set()}')
        t_end = time.time() + duration
        while True:

            if self.detect == 1:
                self.eventObjectDetected.set()
                print(f'[DEBUG scan output] {self.eventObjectDetected.is_set()}')
                print('[DEBUG] event object detected set')

            if time.time() > t_end:
                print('[DEBUG scan output] duration done break loop ')
                self.eventSendScanIndicator.clear()
                print(f'[DEBUG scan output] Scan indicator: {self.eventSendScanIndicator.is_set()}')
                break

