from Drone import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin
import time                              # import time library
import threading
from vision import DroneCamVision

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(position.lat, position.lon)

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

vision = DroneCamVision(1234)
vision.model_setup()

# lats = [-35.36311393, -35.36265179, -35.36266860, -35.36309214, -35.36355729]     # latitudes of plant locations
# longs = [149.16456640, 149.16401228, 149.16345636, 149.16293594, 149.16460797]    # longitudes of plant locations
lats = [-35.36311393, -35.36265179]
longs = [149.16456640, 149.16401228]

alt = 4                # set altitude (m)
airspeed = 5           # set airspeed (m/s)
n = 0  # way point increment
way_point = []

while True:

    # initiates event flags
    TO = Hex.eventTakeOffComplete.is_set()
    MC = Hex.eventMissionComplete.is_set()
    TA = Hex.eventThreadActive.is_set()
    LR = Hex.eventLocationReached.is_set()
    OD = vision.eventObjectDetected.is_set()
    SC = Hex.eventScanComplete.is_set()
    P = Hex.eventPlant.is_set()
    DTA = Hex.eventDistanceThreadActive.is_set()

    # when n is incremented the next way point is set or mission is complete
    if n <= (len(lats)-1):
        way_point = Hex.get_plant_location(lats[n], longs[n], alt)
    else:
        Hex.eventMissionComplete.set()

    if not DTA and not LR:
        distance_check = threading.Thread(target=Hex.check_distance, args=[way_point])
        distance_check.start()
        DTA = Hex.eventDistanceThreadActive.set()
        print('set distance thread')

    if not MC and not TO and not TA:
        # arm drone and take off using method from Drone.py, passing specified altitude as argument
        take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[alt])
        take_off.start()
        print('taking off')

    if not MC and TO and not TA and not LR:
        fly = threading.Thread(target=Hex.fly_to_point, args=([way_point], airspeed))
        fly.start()

    if not MC and not TA and LR and TO:
        detect = threading.Thread(target=vision.run_detection)
        detect.start()
        scan = threading.Thread(target=Hex.the_only_real_scan, args=[20])
        scan.start()

    if not MC and not TA and OD and LR and SC and TO:
        print('[INFO] Object detected, moving to next way point')
        vision.eventObjectDetected.clear()
        Hex.eventScanComplete.clear()
        Hex.eventLocationReached.clear()
        n += 1

    if not MC and not TA and not OD and LR and SC and TO:
        plant = threading.Thread(target=Hex.set_plant_flag)
        plant.start()
        print('planting')
        time.sleep(3)

    if P:
        n += 1
        Hex.eventPlant.clear()
        Hex.eventLocationReached.clear()

    if MC and TO and not TA:
        complete_mission = threading.Thread(target=Hex.return_home)
        complete_mission.start()
        print('returning to home location')
        break

