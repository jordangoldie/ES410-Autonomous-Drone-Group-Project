from Drone import Drone
import time
from data_logging import DataLogging

print("Starting program!")

Hex = Drone("127.0.0.1:14550")

InFlightLogging = DataLogging()
InFlightLogging.PrepLogging()

InFlightLogging.InfoLogging()
Hex.arm_and_takeoff(4)
InFlightLogging.InfoLogging()
Hex.fly_too_point(10, 20, 6)
InFlightLogging.InfoLogging()
Hex.return_home()
InFlightLogging.InfoLogging()

InFlightLogging.FinishLogging()

print("Program finished!")