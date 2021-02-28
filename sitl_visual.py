from Drone2 import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin, get_vector
import time
import threading
from vision import DroneCamVision

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

Hex.origin = set_origin(-35.36355729, 149.16460797)
print('[INFO MAIN] >> Origin for GPS transforms set')

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

vision_communication = threading.Thread(target=Hex.handle_vision)
vision_communication.start()

command = input('[INFO MAIN] >> PRESS ANY KEY TO CONTINUE')

airspeed = 5
alt = 3
duration = 20
lats = [-35.36311393, -35.36265179]
longs = [149.16456640, 149.16401228]
way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], alt))
TO = 0

radius = 2

while True:

    if TO == 0:
        take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[3])
        take_off.start()
        TO = 1

    command = input('Give command:')

    if command == 'circle':
        location = Hex.get_plant_location(-35.36355729, 149.16460797, 3)
        scan = threading.Thread(target=Hex.scan, args=(location, 40, radius))
        scan.start()

    elif command == 'detect':
        '''vision.tcp.send_message('1')
        vision.run_detection(20)'''

    elif command == 'scan':
        location = Hex.get_plant_location(-35.36355729, 149.16460797, 3)
        scan = threading.Thread(target=Hex.scan, args=[location, duration, radius])
        scan.start()
        run_detection = threading.Thread(target=Hex.scan_output, args=[duration])
        run_detection.start()

    elif command == 'fly':
        Hex.send_global_velocity(4, 4, 0, 10)

    elif command == 'fly x':
        Hex.send_global_velocity(4, 0, 0, 5)

    elif command == 'fly y':
        Hex.send_global_velocity(0, 4, 0, 5)

    elif command == 'fly z':
        Hex.send_global_velocity(0, 0, 4, 5)

    elif command == 'fly to way point':
        wp = Hex.get_plant_location(lats[0], longs[0], 10)
        Hex.fly_to_point(wp, airspeed)

