import dronekit as dk
from dronekit import connect
from dronekit import VehicleMode
import time
import math
import socket


class Drone:
    def __init__(self, connection_str):
        try:
            self.vehicle = connect(connection_str)
        except dk.APIException:
            print("Timeout")
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def bind_server_socket(self, host_ip, port):
        host_name = socket.gethostname() # gets name of host machine
        host_ip = socket.gethostbyname(host_name) # gets IP of host machine
        port = 1234
        server_address = (host_ip, port)
        self.server_socket.bind(server_address) # binds to socket
        self.server_socket.listen(5) # listens for TCP connections
        print("[INFO] NOW LISTENING AT HOST ADDRESS: ", server_address)

    def listen_for_tcp(self):
        self.server_socket.listen(5)
        print("[INFO] NOW LISTENING AT: ", server_address)
        client_socket, client_address = self.server_socket.accept()
        print("[INFO] GOT CONNECTION FROM: ", client_address)

    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
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

    def distance_to_point_m(self, point):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        Modified from dronekit example documentation
        The final term deals with the earths curvature
        """
        dlat = point.lat - self.vehicle.location.global_relative_frame.lat
        dlong = point.lon - self.vehicle.location.global_relative_frame.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
        string = f'distance: {distance}, point: {point}, current: {self.vehicle.location.global_relative_frame}'
        return string

    def distance_to_point(self, point):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        Modified from dronekit example documentation
        The final term deals with the earths curvature
        """
        dlat = point.lat - self.vehicle.location.global_relative_frame.lat
        dlong = point.lon - self.vehicle.location.global_relative_frame.lon
        distance = math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
        #string = f'distance: {distance}, point {point}, dlat: {dlat}, dlong: {dlong}'
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
        self.vehicle.airspeed = airspeed
        print("Flying towards point")
        self.vehicle.simple_goto(location)

    def plant_wait(self, plant_time):
        for i in range(plant_time):
            print("planting...")
            time.sleep(1)

    def set_plant_flag(self):
        return 1

    def return_home(self):
        self.vehicle.mode = VehicleMode("RTL")

    def get_home_location(self):
        home = self.vehicle.home_location
        return home


