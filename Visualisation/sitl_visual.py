from Drone2 import Drone                  # import Drone class from Drone.py
from GPS import set_origin, get_vector
import threading

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

unity = threading.Thread(target=Hex.handle_unity) # handles unity tcp and positional data
unity.start()

command = input('[INFO MAIN] >> PRESS ANY KEY TO CONTINUE') # pause for entered key

airspeed = 5
alt = 3
duration = 20
TO = 0

radius = 2

while True:

    # take off once entering loop
    if TO == 0:
        take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[2.5])
        take_off.start()
        TO = 1

    # allows for any of the functions below to be tested in the visualiser
    command = input('Give command:')

    if command == 'circle':
        location = Hex.get_plant_location(52.38255, -1.56156, 3)
        scan = threading.Thread(target=Hex.scan, args=(location, 40, radius))
        scan.start()

    elif command == 'home':
        location = Hex.get_plant_location(-35.36355729, 149.16460797, 2.5)
        Hex.fly_to_point(location, 1)

    elif command == 'scan':
        location = Hex.get_plant_location(-35.36355729, 149.16460797, 2.5)
        scan = threading.Thread(target=Hex.scan, args=[location, duration, radius])
        scan.start()
        run_detection = threading.Thread(target=Hex.scan_output, args=[duration])
        run_detection.start()

    elif command == 'scan3':
        location = Hex.get_plant_location(-35.36355729, 149.16460797, 2.5)
        scan = threading.Thread(target=Hex.scan, args=[location, duration, 3])
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

