from old.Drone import Drone                  # import Drone class from Drone.py
from GPS import set_origin
import threading
from vision import DroneCamVision

Hex = Drone("127.0.0.1:14550") # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

Hex.origin = set_origin(-35.36355729, 149.16460797)
print('[INFO MAIN] >> Origin for GPS transforms set')

unity = threading.Thread(target=Hex.handle_unity)
unity.start()
print('[INFO MAIN] >> Handle unity thread active')

vision = DroneCamVision(1234)
vision.model_setup()
print('[INFO MAIN] >> Vision object initiated')

lats= [52.38245, 52.38372]  # latitudes of plant locations
longs = [-1.56157, -1.55861]

alt = 2.5
plant_indicators = [0, 1, 0, 1, 0]
airspeed = 5           # set airspeed (m/s)


way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], alt))
n = 0  # way point increment
print('[INFO MAIN] >> Way points loaded')

print('[INFO MAIN] >> Entering mission loop')
while True:

    # initiates event flags
    TO = Hex.eventTakeOffComplete.is_set()
    MC = Hex.eventMissionComplete.is_set()
    TA = Hex.eventThreadActive.is_set()
    LR = Hex.eventLocationReached.is_set()
    SC = Hex.eventScanComplete.is_set()
    P = Hex.eventPlantComplete.is_set()
    DTA = Hex.eventDistanceThreadActive.is_set()

    # when n is incremented the next way point is set or mission is complete
    if n == len(way_points)-1:
        Hex.eventMissionComplete.set()

    if not MC:
        if not TO and not TA:
            # arm drone and take off using method from Drone.py, passing specified altitude as argument
            take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[alt])
            take_off.start()
            print('[INFO MAIN] >> Armed, take off initiated')

        if TO and not TA:
            if not LR:
                fly_to = threading.Thread(target=Hex.fly_to_point, args=(way_points[n], airspeed))
                fly_to.start()
                print('[INFO MAIN] >> flying to way point')

            if LR and not SC:
                if plant_indicators[n] == 0:
                    n += 1
                    print("[INFO MAIN] >> way point not plant location")
                    Hex.eventLocationReached.clear()
                elif plant_indicators[n] == 1:
                    print("[INFO MAIN] >> plant location reached")
                    scan = threading.Thread(target=Hex.scan, args=(20, 2))
                    scan.start()
                    vision.tcp.send_message('1')
                    run_detection = threading.Thread(target=vision.run_detection, args=[25])
                    run_detection.start()
                    print("[INFO MAIN] >> Scanning plant location for people")

            if SC and not P:
                if vision.eventObjectDetected:
                    '[INFO MAIN] >> Planting aborted'
                    n += 1
                    vision.eventObjectDetected.clear()
                    vision.tcp.send_message('0')
                else:
                    plant = threading.Thread(target=Hex.set_plant_flag)
                    plant.start()
                    vision.eventObjectDetected.clear()
                    vision.tcp.send_message('0')

            if P:
                n += 1
                Hex.eventPlantComplete.clear()
                Hex.eventScanComplete.clear()
                Hex.eventLocationReached.clear()

    if MC and TO and not TA:
        complete_mission = threading.Thread(target=Hex.return_home)
        complete_mission.start()
        print('[INFO MAIN] >> returning to home location')
        break
