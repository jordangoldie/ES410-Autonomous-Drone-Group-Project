from Drone_event import Drone                  # import Drone class from Drone.py
import time                                    # import time library
import threading                               # import threading library

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

lats = [52.38345, 52.38372]      # set latitude
longs = [-1.56009, -1.55861]     # set longitude   
plant_flags = [1, 1, 0, 1]       # set plant flags
flying_alt = 8                   # set flying altitude (m)
plant_alt = 3                    # set planting altitude (m)
airspeed = 4                     # set airspeed (m/s)
plant_count = len(lats)          # set number of planting locations
duration = 30                    # set circle duration
radius = 2                       # set circle radius
n = 0                            # way point increment

# generate list of waypoints
way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], flying_alt))

# start take off thread 
take_off = threading.Thread(target=Hex.arm_and_takeoff, args=(flying_alt))
take_off.start()
print('[INFO MAIN] >> Armed, take off initiated')

# in-flight infinite loop 
while True:
    
    # if all locations travelled to, break from loop
    if Hex.waypoint_count == len(way_points):
        Hex.eventMissionComplete.set()
        break
        
    # if take off complete and thread sequence not active 
    if not Hex.eventThreadSeqActive.is_set() and Hex.eventTakeOffComplete.is_set():
        
        # set thread sequence active flag
        Hex.eventThreadSeqActive.set()
        print('[INFO MAIN] >> entered thread sequence')
        n = Hex.waypoint_count     # set current waypoint 
        
        # fly to thread
        fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed, plant_flags[n]))
        fly_to.start()
        print('[INFO MAIN] >> flying to way point')
        if plant_flags[n] == 1:     # if planting location
            # descend thread
            descend = threading.Thread(target=Hex.descend, args=[plant_alt])
            descend.start()
            print('[INFO MAIN] >> descend thread')
            
            # scan feature threads
            scan = threading.Thread(target=Hex.scan, args=[way_points[n], duration, radius])
            scan.start()
            run_detection = threading.Thread(target=Hex.scan_output, args=[30])
            run_detection.start()
            print('[INFO MAIN] >> scan thread')
            
            # plant thread
            plant = threading.Thread(target=Hex.set_plant_flag)
            plant.start()
            print('[INFO MAIN] >> plant thread')
            
            # ascend thread
            ascend = threading.Thread(target=Hex.ascend, args=[flying_alt])
            ascend.start()
            print('[INFO MAIN] >> ascend thread')
       

    time.sleep(2)     # pause 


complete_mission = threading.Thread(target=Hex.return_home)
complete_mission.start()
print('[INFO MAIN] >> complete mission thread')

