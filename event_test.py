from Drone_Event import Drone  # import Drone class from Drone.py
from TCP import TCP
from GPS2 import set_origin, get_vector
from data_logging import DataLogging  # import DataLogging class from data_logging.py
import time  # import time library
import threading
import argparse


class ThreadSeqFlagDetector:
    """Detects False to True transitions on an external signal."""

    def __init__(self, reader, action):
        self.reader = reader
        self.action = action
        self.last_value = reader()    # initialise value

    def test(self):
        new_value = self.reader()
        if new_value and not self.last_value:
            self.action()
        self.last_value = new_value


def flag_reader():  # generate test sequence
    value = Hex.eventTakeOffComplete.is_set()
    print("Read:", value)
    return value


def action():
    Hex.fly_to_point(way_points[0], airspeed, plant_flags[0])


Hex = Drone("127.0.0.1:14550")  # Create instance of drone class, passing IP and Port for UDP socket
position = Hex.get_current_location()
# Hex.origin = set_origin(position.lat, position.lon)

lats = [-35.36311393, -35.36265179, -35.36266860, -35.36309214, -35.36355729]  # latitudes of plant locations
longs = [149.16456640, 149.16401228, 149.16345636, 149.16293594, 149.16460797]  # longitudes of plant locations
plant_flags = [0, 1, 1, 0, 0]

alt = 4  # set altitude (m)
airspeed = 5  # set airspeed (m/s)

n = 0  # way point increment
way_points = []
for i in range(len(lats)):
    way_points.append(Hex.get_plant_location(lats[i], longs[i], alt))

plant_indicator = 0
plant_count = len(lats)

take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[alt])
take_off.start()
print("take off thread")

flagDetector = ThreadSeqFlagDetector(flag_reader, action)
while True:
    flagDetector.test()