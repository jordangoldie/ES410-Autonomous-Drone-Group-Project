from Drone import Drone
from data_logging import DataLogging
import time
import socket

Hex = Drone("127.0.0.1:14550")

# create socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
print('[INFO] HOST IP: ', host_ip)
port = 5599
server_address = (host_ip, port)
# bind socket
# server_socket.bind(('localhost', 1234))
server_socket.bind(server_address)
# socket listen
server_socket.listen(5)
print("[INFO] NOW LISTENING AT: ", server_address)

while True:
    client_socket, address = server_socket.accept()
    print("[INFO] GOT CONNECTION FROM: ", address)
    if client_socket:
        break

client_socket.send(bytes('welcome to the SITL', "utf-8"))

lats = [-35.36265179, -35.36266860, -35.36309214, -35.36355729, -35.36354127]
longs = [149.16401228, 149.16345636, 149.16293594, 149.16460797, 149.16399818]

alt = 4
airspeed = 5

plant_count = 3
plant_flag = 0
plant_time = 5

InFlightLogging = DataLogging()
InFlightLogging.PrepLogging()
InFlightLogging.InfoLogging(Hex)

Hex.arm_and_takeoff(alt)
lat = Hex.get_current_location().lat
lon = Hex.get_current_location().lon
alt = Hex.get_current_location().alt
client_socket.send(bytes(f'{lat},{lon},{alt}', "utf-8"))


for i in range(plant_count):
    plant_location = Hex.get_plant_location(lats[i], longs[i], alt)
    current_location = Hex.get_current_location()
    Hex.fly_to_point(plant_location, airspeed)
    distance = Hex.distance_to_point_m(plant_location)

    while distance >= 1:
        time.sleep(1)
        distance = Hex.distance_to_point_m(plant_location)
        print(distance)
        InFlightLogging.InfoLogging(Hex)

        lat = Hex.get_current_location().lat
        lon = Hex.get_current_location().lon
        alt = Hex.get_current_location().alt

        client_socket.send(bytes(f'{lat},{lon},{alt}', "utf-8"))

    plant_flag = Hex.set_plant_flag()
    print("plant flag:", plant_flag)
    Hex.plant_wait(5)


Hex.return_home()
