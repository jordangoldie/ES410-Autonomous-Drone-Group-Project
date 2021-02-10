from Drone import Drone                  # import Drone class from Drone.py
from data_logging import DataLogging     # import DataLogging class from data_logging.py
import time                              # import time library
import threading

Hex = Drone("127.0.0.1:14550")     # Create instance of drone class, passing UDP port number for SITL connection

lats = [-35.36311393, -35.36265179, -35.36266860, -35.36309214, -35.36355729]     # latitudes of plant locations
longs = [149.16456640, 149.16401228, 149.16345636, 149.16293594, 149.16460797]    # longitudes of plant locations

alt = 4                # set altitude (m)
airspeed = 5           # set airspeed (m/s)
plant_count = 0        # set plant count
num_plants = 1
plant_flag = 0         # set plant flag
plant_duration = 5     # set duration of planting (s)
plant_range = 1

plant_location1 = Hex.get_plant_location(lats[plant_count], longs[plant_count], alt)
plant_location2 = Hex.get_plant_location(lats[1], longs[1], alt)

# Hex.arm_and_takeoff(alt) # arm drone and take off using method from Drone.py, passing specified altitude as argument

while True:

    TO = Hex.eventTakeOffComplete.is_set()
    MC = Hex.eventMissionComplete.is_set()
    TA = Hex.eventThreadActive.is_set()
    LR = Hex.eventLocationReached.is_set()
    OD = Hex.eventObjectDetected.is_set()
    SC = Hex.eventScanComplete.is_set()
    P = Hex.eventPlant.is_set()
    # DTA = Hex.eventDistanceThreadActive.is_set()

    way_point = Hex.get_plant_location(lats[0], longs[0], alt)


    '''if not DTA:
        distance_check = threading.Thread(target=Hex.check_distance(plant_location1))
        print('Debug')
        DTA = Hex.eventDistanceThreadActive.set()
        print('set distance thread')'''

    if not MC and not TO and not TA:
        take_off = threading.Thread(target=Hex.arm_and_takeoff(alt))
        print('taking off')

    if not MC and not TA and not LR and TO:
        fly_to = threading.Thread(target=Hex.fly_to_point(way_point))
        print('flying to')

    if not MC and not TA and LR and TO:
        plant = threading.Thread(target=Hex.set_plant_flag)
        print('planting')
        time.sleep(3)

    if P:
        break

    # if MC and TO and not TA:
    #   complete_mission = threading.Thread(target=Hex.return_home)


Hex.return_home()

