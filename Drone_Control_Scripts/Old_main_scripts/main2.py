from Drone2 import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin
import time                              # import time library
import threading
from vision import DroneCamVision

Hex = Drone("127.0.0.1:14550") # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(52.38255, -1.56156)

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

vision_communication = threading.Thread(target=Hex.handle_vision)
vision_communication.start()

time.sleep(10)

lats= [52.38345, 52.38372, 52.38399, 52.38525, 52.38704, 52.37086]
longs = [-1.56009, -1.55861, -1.56436, -1.56274, -1.55420, -1.56892]

alts = [3, 3, 3, 3, 3]
plant_indicators = [0, 1, 0, 1, 0]

way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], alts[i]))
n = 0  # way point increment

airspeed = 5           # set airspeed (m/s)

while True:

    # initiates event flags
    TO = Hex.eventTakeOffComplete.is_set()
    MC = Hex.eventMissionComplete.is_set()
    TA = Hex.eventThreadActive.is_set()
    LR = Hex.eventLocationReached.is_set()
    OD = Hex.eventObjectDetected.is_set()
    SC = Hex.eventScanComplete.is_set()
    P = Hex.eventPlantComplete.is_set()
    DTA = Hex.eventDistanceThreadActive.is_set()

    # when n is incremented the next way point is set or mission is complete
    if n == len(way_points)-1:
        Hex.eventMissionComplete.set()

    if not MC:
        if not TO and not TA:
            # arm drone and take off using method from Drone.py, passing specified altitude as argument
            take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[alts[n]])
            take_off.start()
            print('[INFO MAIN] Armed, take off initiated')

        if TO and not TA:
            if not LR:
                fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed))
                fly_to.start()
                print('[INFO MAIN] flying to')

            if LR and not SC:
                if plant_indicators[n] == 0:
                    n += 1
                    print("[INFO MAIN] not plant location")
                    Hex.eventLocationReached.clear()
                elif plant_indicators[n] == 1:
                    scan = threading.Thread(target=Hex.scan, args=(20, 2))
                    run_detection = threading.Thread(target=Hex.scan_output, args=[30])
                    scan.start()
                    run_detection.start()

            if SC and not P:
                if OD:
                    '[INFO MAIN] Abandon plant location'
                    n += 1
                    Hex.eventObjectDetected.clear()
                else:
                    plant = threading.Thread(target=Hex.set_plant_flag)
                    plant.start()
                    Hex.eventObjectDetected.clear()

            if P:
                n += 1
                Hex.eventPlantComplete.clear()
                Hex.eventScanComplete.clear()
                Hex.eventLocationReached.clear()

    if MC and TO and not TA:
        complete_mission = threading.Thread(target=Hex.return_home)
        complete_mission.start()
        print('[INFO MAIN] returning to home location')
        break
