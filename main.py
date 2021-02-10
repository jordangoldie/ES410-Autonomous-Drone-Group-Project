from Drone import Drone                  # import Drone class from Drone.py
from data_logging import DataLogging     # import DataLogging class from data_logging.py
import time                              # import time library
import threading

Hex = Drone("127.0.0.1:14550")     # Create instance of drone class, passing UDP port number for SITL connection

lats = [-35.36311393, -35.36265179, -35.36266860, -35.36309214, -35.36355729]     # latitudes of plant locations
longs = [149.16456640, 149.16401228, 149.16345636, 149.16293594, 149.16460797]    # longitudes of plant locations

alt = 4                # set altitude (m)
airspeed = 5           # set airspeed (m/s)
plant_count = 0        # set plant count
num_plants = 1
plant_flag = 0         # set plant flag
plant_duration = 5     # set duration of planting (s)
plant_range = 1

plant_location1 = Hex.get_plant_location(lats[plant_count], longs[plant_count], alt)
plant_location2 = Hex.get_plant_location(lats[1], longs[1], alt)

Hex.arm_and_takeoff(alt)     # arm drone and take off using method from Drone.py, passing specified altitude as argument

while True:
    if Hex.eventTakeOff.is_set():
        t1 = threading.Thread(target=Hex.fly_to_point(plant_location1, airspeed))
        t2 = threading.Thread(target=Hex.check_distance(plant_location1))
        Hex.eventTakeOff.clear()
    if Hex.eventPlantLocationReached.is_set():
        t3 = threading.Thread(target=Hex.fly_to_point2(plant_location2,airspeed))
        Hex.eventPlantLocationReached.clear()

#    break
print("loop complete")


'''
while plant_count <= num_plants:
    event = Hex.event()
    if event == "take off complete":
        plant_location = Hex.get_plant_location(lats[plant_count], longs[plant_count], alt)  # get gps location of next planting location
        current_location = Hex.get_current_location()  # get current gps location of drone
        distance = Hex.distance_to_point_m(plant_location)  # get distance of drone to plant location in m
        Hex.fly_to_point(plant_location, airspeed)  # fly to plant location at specified airspeed

        while True:  # keep drone flying to plant location until within 1 m
            distance = Hex.check_distance(plant_location, plant_range)  # get updated distance to plant location in  m
            print(distance)  # print distance
            time.sleep(1)
            if distance <= 1:
                break

        plant_count += 1

    elif event == "location reached":
        plant_location = Hex.get_plant_location(lats[plant_count], longs[plant_count],
                                                alt)  # get gps location of next planting location
        current_location = Hex.get_current_location()  # get current gps location of drone
        distance = Hex.distance_to_point_m(plant_location)  # get distance of drone to plant location in m
        Hex.fly_to_point(plant_location, airspeed)  # fly to plant location at specified airspeed

        while True:  # keep drone flying to plant location until within 1 m
            distance = Hex.distance_to_point_m(plant_location)  # get updated distance to plant location in  m
            print(distance)  # print distance
            time.sleep(1)
            if distance <= 1:
                break

        plant_count += 1


 #   plant_flag = Hex.set_plant_flag()     # set plant flag to 1 for planting with arduino code
 #   print("plant flag:", plant_flag)      # print plant flag
 #   Hex.plant_wait(plant_duration)        # wait for planting process to complete


Hex.return_home()     # run method from Drone.py to get drone to fly back to starting position and land
'''