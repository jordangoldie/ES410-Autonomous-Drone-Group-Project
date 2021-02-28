from Drone_event import Drone                  # import Drone class from Drone.py
from Data_logging import DataLogging     # import DataLogging class from Data_logging.py
import time                              # import time library
import threading

Hex = Drone("127.0.0.1:14550")    # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

data = DataLogging()

data.PrepLogging()

take_off = threading.Thread(target=Hex.arm_and_takeoff, args=[10])
take_off.start()

while True:
    time.sleep(0.001)
    data.InfoLogging(Hex)