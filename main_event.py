from Drone_event import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin, get_vector
from Data_logging import DataLogging     # import DataLogging class from Data_logging.py
import time                              # import time library
import threading
import argparse

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(position.lat, position.lon)

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

lats = [-35.36311393, -35.36265179, -35.36266860, -35.36355729]     # latitudes of plant locations
longs = [149.16456640, 149.16401228, 149.16345636, 149.16460797]    # longitudes of plant locations

plant_flags = [1, 1, 0, 1]
flying_alt = 8
plant_alt = 3
airspeed = 5           # set airspeed (m/s)
plant_count = len(lats)

way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], flying_alt))
n = 0  # way point increment

#take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[flying_alt])
#take_off.start()
#print("take off thread")

Hex.arm_and_takeoff(flying_alt)
way_points.append(Hex.get_current_location())
plant_flags.append(0)

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
            descend = threading.Thread(target=Hex.descend, args=[plant_alt])
            descend.start()
            print("descend thread")
            scan = threading.Thread(target=Hex.scan, args=[0])
            scan.start()
            print("scan thread")
            plant = threading.Thread(target=Hex.set_plant_flag)
            plant.start()
            print("plant thread")
            ascend = threading.Thread(target=Hex.ascend, args=[flying_alt])
            ascend.start()
            print("ascend thread")
        else:
            Hex.waypoint_count += 1

    time.sleep(2)


time.sleep(10)
complete_mission = threading.Thread(target=Hex.return_home)
complete_mission.start()
print("complete mission thread")

