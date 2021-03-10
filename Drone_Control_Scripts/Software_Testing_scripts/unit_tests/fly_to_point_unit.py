from Drone_event_unit import Drone  # import Drone class from Drone_event.py
from GPS_unit import set_origin
from datetime import datetime
import threading

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
Hex.origin = set_origin(-35.36355729, 149.16460797)
print('[INFO MAIN] >> UDP connection to SITL established')

lat = -35.36312083     # latitude
lon = 149.16457298     # longitude
flying_alt = 15        # specify target altitude (m)
airspeed = 5          # set airspeed (m/s)
plant_flag = 0        # plant flag (0 = not planting location)

waypoint = Hex.get_plant_location(lat, lon, flying_alt)    # generate location object
Hex.arm_and_takeoff(flying_alt)                                    # run arm and take off function
Hex.fly_to_point(waypoint, airspeed, plant_flag)           # run fly to point function








