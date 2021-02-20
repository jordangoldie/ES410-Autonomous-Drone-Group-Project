from Drone import Drone                  # import Drone class from Drone.py
from TCP import TCP
from GPS2 import set_origin
import time
import threading

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
Hex.origin = set_origin(position.lat, position.lon)

'''tcp = TCP(5598)  # create instance of tcp class
tcp.bind_server_socket()
# if args["unity"] == 0:
tcp.listen_for_tcp()'''

unity = threading.Thread(target=Hex.handle_unity)
unity.start()

airspeed = 5

TO = 0

while True:

    if TO == 0:
        take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[10])
        take_off.start()
        TO = 1

    command = input('Give command:')

    if command == 'circle':
        Hex.circle(20)
    elif command == 'fly':
        Hex.send_global_velocity(4, 4, 0, 10)
