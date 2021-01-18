from Drone import Drone
import time

Hex = Drone("127.0.0.1:14550")

lats = [-35.36265179, -35.36266860, -35.36265183]
longs = [149.16401228, 149.16345636, 149.16294162]

lat = -35.36311962
lon = 149.16460948
alt = 4
airspeed = 5

Hex.arm_and_takeoff(alt)
plant_location = Hex.get_plant_location(lat, lon, alt)
current_location = Hex.get_current_location()
Hex.fly_to_point(plant_location, airspeed)
distance = Hex.distance_to_point_m(plant_location)

while distance >= 1:
    time.sleep(1)
    distance = Hex.distance_to_point_m(plant_location)
    print(distance)

Hex.return_home()
