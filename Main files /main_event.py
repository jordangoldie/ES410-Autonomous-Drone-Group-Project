from Drone_event import Drone                  # import Drone class from Drone.py
import time                              # import time library
import threading

# Create instance of drone class, passing IP and Port for UDP socket
Hex = Drone("127.0.0.1:14550")
print('[INFO MAIN] >> UDP connection to SITL established')

# creates thread that sends position and orientation data to unity
unity = threading.Thread(target=Hex.handle_unity)
unity.start()

# creates a thread that handles the communication with the external_vision.py file
vision_communication = threading.Thread(target=Hex.handle_vision)
vision_communication.start()
print('[INFO MAIN] >> TCP connections established')

# pause here until something is entered in the command window
command = input('[INFO MAIN] >> PRESS ANY KEY TO CONTINUE')
print('[INFO MAIN] >> Starting flight sequence')

# set lat/longs
lats = [52.38345, 52.38372]
longs = [-1.56009, -1.55861]

plant_flags = [1, 1, 0, 1]
flying_alt = 8
plant_alt = 3
airspeed = 4           # set airspeed (m/s)
plant_count = len(lats)
duration = 30
radius = 2
way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], flying_alt))
n = 0  # way point increment

Hex.arm_and_takeoff(flying_alt)
print('[INFO MAIN] >> Armed, take off initiated')
way_points.append(Hex.get_current_location())
plant_flags.append(0)

while True:

    # print("active count:", threading.active_count())
    if Hex.waypoint_count == len(way_points):
        break

    if not Hex.eventThreadSeqActive.is_set() and Hex.eventTakeOffComplete.is_set():
        Hex.eventThreadSeqActive.set()
        print('[INFO MAIN] >> entered thread sequence')
        n = Hex.waypoint_count
        fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed, plant_flags[n]))
        fly_to.start()
        print('[INFO MAIN] >> flying to way point')
        if plant_flags[n] == 1:
            descend = threading.Thread(target=Hex.descend, args=[plant_alt])
            descend.start()
            print('[INFO MAIN] >> descend thread')
            scan = threading.Thread(target=Hex.scan, args=[way_points[n], duration, radius])
            scan.start()
            run_detection = threading.Thread(target=Hex.scan_output, args=[30])
            run_detection.start()
            print('[INFO MAIN] >> scan thread')
            plant = threading.Thread(target=Hex.set_plant_flag)
            plant.start()
            print('[INFO MAIN] >> plant thread')
            ascend = threading.Thread(target=Hex.ascend, args=[flying_alt])
            ascend.start()
            print('[INFO MAIN] >> ascend thread')
        else:
            Hex.waypoint_count += 1

    time.sleep(2)

time.sleep(10)
complete_mission = threading.Thread(target=Hex.return_home)
complete_mission.start()
print('[INFO MAIN] >> complete mission thread')

