from Drone import Drone                  # import Drone class from Drone.py
from data_logging import DataLogging     # import DataLogging class from data_logging.py
import time                              # import time library

Hex = Drone("127.0.0.1:14550")     # Create instance of drone class, passing UDP port number for SITL connection

lats = [-35.36265179, -35.36266860, -35.36309214, -35.36355729, -35.36354127]     # latitudes of plant locations
longs = [149.16401228, 149.16345636, 149.16293594, 149.16460797, 149.16399818]    # longitudes of plant locations

alt = 4                # set altitude (m)
airspeed = 5           # set airspeed (m/s)
plant_count = 3        # set plant count
plant_flag = 0         # set plant flag
plant_duration = 5     # set duration of planting (s)

InFlightLogging = DataLogging()      # create object for data logging from data_logging.py
InFlightLogging.PrepLogging()        # run method to prep for logging from data_logging.py
InFlightLogging.InfoLogging(Hex)     # run method to log data from data_logging.py, passing Hex object as argument

Hex.arm_and_takeoff(alt)     # arm drone and take off using method from Drone.py, passing specified altitude as argument

# loop controlling drone to to plant at each planting position (given by plant_count and list of longs and lats)
for i in range(plant_count):
    plant_location = Hex.get_plant_location(lats[i], longs[i], alt)     # get gps location of next planting location
    current_location = Hex.get_current_location()                       # get current gps location of drone
    distance = Hex.distance_to_point_m(plant_location)                  # get distance of drone to plant location in m
    Hex.fly_to_point(plant_location, airspeed)                          # fly to plant location at specified airspeed

    while distance >= 1:                                       # keep drone flying to plant location until within 1 m
        distance = Hex.distance_to_point_m(plant_location)     # get updated distance to plant location in  m
        print(distance)                                        # print distance
        InFlightLogging.InfoLogging(Hex)                       # log data
        time.sleep(1)                                          # 1 second delay

    plant_flag = Hex.set_plant_flag()     # set plant flag to 1 for planting with arduino code
    print("plant flag:", plant_flag)      # print plant flag
    Hex.plant_wait(plant_duration)        # wait for planting process to complete


Hex.return_home()     # run method from Drone.py to get drone to fly back to starting position and land
