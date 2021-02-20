from Drone import Drone
from TCP import TCP
from data_logging import DataLogging
from GPS2 import get_vector, set_origin

Hex = Drone("127.0.0.1:14550")  # create instance of drone class
tcp = TCP(5598)  # create instance of tcp class
tcp.bind_server_socket()
client_socket = tcp.listen_for_tcp()

lats = [-35.36265179, -35.36266860, -35.36309214, -35.36355729, -35.36354127]
longs = [149.16401228, 149.16345636, 149.16293594, 149.16460797, 149.16399818]

airspeed = 5

plant_count = 3
plant_flag = 0
plant_time = 5

InFlightLogging = DataLogging()
InFlightLogging.PrepLogging()
InFlightLogging.InfoLogging(Hex)

lat = Hex.get_current_location().lat
lon = Hex.get_current_location().lon
alt = Hex.get_current_location().alt
roll = Hex.get_attitude().roll
pitch = Hex.get_attitude().pitch
yaw = Hex.get_attitude().yaw

# sets the origin based on above 'get', would be better if this was done with home location
pose = set_origin(lat, lon)
vector_fn = get_vector(pose, lat, lon, alt)
string = f'{vector_fn[0]},{vector_fn[1]},{vector_fn[2]},{roll},{pitch},{yaw}'

if client_socket:
    client_socket.send(bytes(string, "utf-8"))

Hex.arm_and_takeoff(4)

lat = Hex.get_current_location().lat
lon = Hex.get_current_location().lon
alt = Hex.get_current_location().alt
roll = Hex.get_attitude().roll
pitch = Hex.get_attitude().pitch
yaw = Hex.get_attitude().yaw
vector_fn = get_vector(pose, lat, lon, 4)
string = f'{vector_fn[0]},{vector_fn[1]},{vector_fn[2]},{roll},{pitch},{yaw}'
if client_socket:
    client_socket.send(bytes(string, "utf-8"))

for i in range(plant_count):
    plant_location = Hex.get_plant_location(lats[i], longs[i], 4)
    current_location = Hex.get_current_location()
    Hex.fly_to_point(plant_location, airspeed)
    distance = Hex.distance_to_point_m(plant_location)

    while distance >= 1:
        distance = Hex.distance_to_point_m(plant_location)
        print(distance)

        # InFlightLogging.InfoLogging(Hex)
        lat = Hex.get_current_location().lat
        lon = Hex.get_current_location().lon
        alt = Hex.get_current_location().alt
        roll = Hex.get_attitude().roll
        pitch = Hex.get_attitude().pitch
        yaw = Hex.get_attitude().yaw
        # calculates cartesian vector from origin to new point and sends to unity
        vector_fn = get_vector(pose, lat, lon, 4)
        string = f'{vector_fn[0]},{vector_fn[1]},{vector_fn[2]},{roll},{pitch},{yaw}'
        client_socket.send(bytes(string, "utf-8"))

    plant_flag = Hex.set_plant_flag()
    print("plant flag:", plant_flag)
    Hex.plant_wait(5)


Hex.return_home()