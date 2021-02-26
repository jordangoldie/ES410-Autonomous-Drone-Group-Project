from Drone import Drone           # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin, get_vector
import time                              # import time library
import threading
import numpy as np

Hex = Drone("127.0.0.1:14550") # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

Hex.origin = set_origin(-35.36355729, 149.16460797)
print('[INFO MAIN] >> Origin for GPS transforms set')

Hex.arm_and_takeoff(15)

location = Hex.get_plant_location(-34, 148.16, 15)
fly = threading.Thread(target=Hex.fly_to_point, args=(location, 5))

while True:

    lat = Hex.get_current_location().lat
    lon = Hex.get_current_location().lon
    alt = Hex.get_current_location().alt
    vec_local = get_vector(Hex.origin, lat, lon)
    print(f'[GPS] >> Position in local frame: {vec_local}')

    north = Hex.vehicle.location.local_frame.north
    east = Hex.vehicle.location.local_frame.east
    up = -Hex.vehicle.location.local_frame.down
    sitl_vec_local = np.array([north, east, up])
    print(f'[SITL]  >> Position in local frame: {sitl_vec_local}')
