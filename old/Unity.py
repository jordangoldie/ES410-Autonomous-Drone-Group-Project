from Drone import Drone
from data_logging import DataLogging
import time
import socket
from old.GPS import get_vector, set_origin

Hex = Drone("127.0.0.1:14550")

# create socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
# host_ip = '0.0.0.0'
print('[INFO] HOST IP: ', host_ip)
port = 5598
server_address = (host_ip, port)
# bind socket
# server_socket.bind(('localhost', 1234))
server_socket.bind(server_address)
# socket listen
server_socket.listen(5)
print("[INFO] NOW LISTENING AT: ", server_address)

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

# accepts the client socket object from unity (must press play on unity)
while True:
    print('[INFO] Accepting...')
    client_socket, address = server_socket.accept()
    print("[INFO] GOT CONNECTION FROM: ", address)
    if client_socket:
        break

lat = Hex.get_current_location().lat
lon = Hex.get_current_location().lon
alt = Hex.get_current_location().alt

# sets the origin based on above 'get', would be better if this was done with home location
pose, rot = set_origin(lat, lon)

if client_socket:
    client_socket.send(bytes(f'{lat},{lon},{alt}', "utf-8"))

Hex.arm_and_takeoff(alt)

for i in range(plant_count):
    plant_location = Hex.get_plant_location(lats[i], longs[i], alt)
    current_location = Hex.get_current_location()
    Hex.fly_to_point(plant_location, airspeed)
    distance = Hex.distance_to_point_m(plant_location)

    while distance >= 1:
        time.sleep(1)
        string = Hex.distance_to_point_m(plant_location)
        print(string)

        InFlightLogging.InfoLogging(Hex)
        lat = Hex.get_current_location().lat
        lon = Hex.get_current_location().lon
        alt = Hex.get_current_location().alt

        # calculates cartesian vector from origin to new point and sends to unity
        unity_vec = get_vector(pose, rot, lat, lon)
        client_socket.send(bytes(unity_vec, "utf-8"))

    plant_flag = Hex.set_plant_flag()
    print("plant flag:", plant_flag)
    Hex.plant_wait(5)


Hex.return_home()