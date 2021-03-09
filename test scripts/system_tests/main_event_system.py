from Drone_event_system import Drone       # import Drone class from Drone.py
from GPS_system import set_origin          # import set_origin method from GPS.py
import time                         # import time library
from datetime import datetime
import threading                    # import threading library

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

Hex.origin = set_origin(-35.36355729, 149.16460797)
print('[INFO MAIN] >> Origin for GPS transforms set')

lats = [52.38299565, 52.38331648, 52.38343311, 52.38389979]     # latitudes of plant locations
longs = [-1.56324699, -1.56326288, -1.56259389, -1.56288053]    # longitudes of plant locations
plant_flags = [0, 1, 0, 1]
flying_alt = 15
plant_alt = 3
airspeed = 4                 # set airspeed (m/s)
plant_count = len(lats)

way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], flying_alt))

n = Hex.waypoint_count

take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[flying_alt])
take_off.start()
print('[INFO MAIN] >> Armed, take off initiated')

while True:

    if Hex.waypoint_count == len(way_points):
        Hex.eventMissionComplete.set()
        break

    if not Hex.eventThreadSeqActive.is_set() and Hex.eventTakeOffComplete.is_set():
        Hex.eventThreadSeqActive.set()
        print(f'[INFO EVENT MAIN] >> Event: ThreadSeqActive.set(), Timestamp: {datetime.now()}')
        n = Hex.waypoint_count
        fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed, plant_flags[n]))
        fly_to.start()
        if plant_flags[n] == 1:
            descend = threading.Thread(target=Hex.descend, args=[plant_alt])
            descend.start()
            plant = threading.Thread(target=Hex.plant())
            plant.start()
            ascend = threading.Thread(target=Hex.ascend, args=[flying_alt])
            ascend.start()

    time.sleep(2)

complete_mission = threading.Thread(target=Hex.return_home)
complete_mission.start()
print('[INFO MAIN] >> complete mission thread')

