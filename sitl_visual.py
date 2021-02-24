from Drone import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS import set_origin, get_vector
import time
import threading
from vision import DroneCamVision

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(position.lat, position.lon)

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

vision = DroneCamVision(1234)
vision.model_setup()
print('vision init complete')

airspeed = 5
lats = [-35.36311393, -35.36265179]
longs = [149.16456640, 149.16401228]
TO = 0

while True:

    if TO == 0:
        take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[3])
        take_off.start()
        TO = 1

    command = input('Give command:')

    if command == 'circle':
        scan = threading.Thread(target=Hex.the_only_real_scan_shady, args=(20, 2))
        scan.start()

    elif command == 'detect':
        vision.tcp.send_message('1')
        vision.run_detection(20)

    elif command == 'scan':
        scan = threading.Thread(target=Hex.the_only_real_scan_shady, args=(20, 3))
        scan.start()
        vision.tcp.send_message('1')
        vision.run_detection(20)

    if vision.eventObjectDetected.set():
        print('BLUDCLART ppl dem')

    if vision.detect == 1:
        print('BLUDFIYA')

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

