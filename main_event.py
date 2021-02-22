from Drone_Event import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS2 import set_origin, get_vector
from data_logging import DataLogging     # import DataLogging class from data_logging.py
import time                              # import time library
import threading
import argparse

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(position.lat, position.lon)

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

lats = [-35.36311393, -35.36265179, -35.36266860, -35.36309214, -35.36355729]     # latitudes of plant locations
longs = [149.16456640, 149.16401228, 149.16345636, 149.16293594, 149.16460797]    # longitudes of plant locations
alt = 4
plant_flags = [0, 1, 0, 1, 0]

way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], alt))
n = 0  # way point increment

airspeed = 5           # set airspeed (m/s)


plant_count = len(lats)

take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[alt])
take_off.start()
print("take off thread")

while True:
    print("active count:", threading.active_count())
    if Hex.waypoint_count == len(way_points):
        break

    if not Hex.eventThreadSeqActive.is_set() and Hex.eventTakeOffComplete.is_set():
        Hex.eventThreadSeqActive.set()
        print("entered thread sequence")
        n = Hex.waypoint_count
        fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed, plant_flags[n]))
        fly_to.start()
        print("fly to thread")
        if plant_flags[n] == 1:
            scan = threading.Thread(target=Hex.scan, args=[0])
            scan.start()
            print("scan thread")
            plant = threading.Thread(target=Hex.set_plant_flag)
            plant.start()
            print("plant thread")
        else:
            Hex.waypoint_count += 1

    time.sleep(2)

time.sleep(10)
complete_mission = threading.Thread(target=Hex.return_home)
complete_mission.start()
print("complete mission thread")

