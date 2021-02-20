# https://dronekit-python.readthedocs.io/en/latest/guide/vehicle_state_and_parameters.html#vehicle-state-attributes

# ES410 Autonomous Drone
# Owner:
# File: data_logging.py
# Description:
from datetime import datetime
import dronekit
import os
# Havent implemented use of LEDs (gpiozero - simple interface to GPIO devices with Raspberry Pi)
# https://gpiozero.readthedocs.io/en/stable/
import time # for time.sleep(seconds)
# from dronekit import connect # NEEDED?
from collections import OrderedDict
from Drone import Drone # IMPORT JORDANS FI
import dronekit as dk
class DataLogging:
    def __init__ (self):
        self.data_file = None
        self.data = None

    def PrepLogging(self):
        # Save and open file, write file header
        self.data_file = open("datafile_" + datetime.now().strftime("%d-%m-%y_%H-%M-%S") + ".csv", "a+")
        self.data_file.write("Logging started at " + datetime.now().strftime("%d-%m-%y at %H:%M:%S\n"))
        self.data_file.write("Timestamp, Longitude, Latitude, Altitude, Roll, Pitch, Yaw, "
                             "Velocity, Groundspeed, Airspeed, Current, Voltage\n")

    def InfoLogging(self, Hex):
        self.data = OrderedDict()
        print()
        self.data["Timestamp"] = datetime.now().strftime("%H:%M:%S.%f")
        self.data["Location lon"] = str(Hex.get_current_location().lon)
        self.data["Location lat"] = str(Hex.get_current_location().lat)
        self.data["Location alt"] = str(Hex.get_current_location().alt)
        self.data["Roll"] = str(Hex.get_attitude().roll)
        self.data["Pitch"] = str(Hex.get_attitude().pitch)
        self.data["Yaw"] = str(Hex.get_attitude().yaw)
        self.data["Velocity"] = str(Hex.get_velocity())
        self.data["Ground speed"] = str(Hex.get_ground_speed())
        self.data["Airspeed"] = str(Hex.get_airspeed())
        self.data["Current"] = str(0)
        self.data["Voltage"] = str(0)
        
        self.data_file.write(', '.join(self.data.values()) + "\n")
    def FinishLogging(self):
        self.data_file.close()

'''
TEST BENCH:
print("Starting program!")
InFlightLogging = DataLogging()
InFlightLogging.PrepLogging()
for i in range(5):
    InFlightLogging.InfoLogging()
    time.sleep(1)
InFlightLogging.FinishLogging()
print("Program finished!")
'''


'''
Location lon # .location.global_frame.lon / .location.global_relative_frame.lon
Location lat # .location.global_frame.lat / .location.global_relative_frame.lat
Location alt # .attitude / .location.global_frame.alt / .location.global_relative_frame.alt
Velocity # .velocity
Ground Speed # .groundspeed
Airspeed # .airspeed
'''

'''
DroneKit: (to pull data)
https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
'''

'''
USB:
https://www.raspberrypi-spy.co.uk/2014/05/how-to-mount-a-usb-flash-disk-on-the-raspberry-pi/
https://rasspberrypi.wordpress.com/2012/09/04/mounting-and-unmounting-usb-drives-on-raspberry-pi/
https://pimylifeup.com/raspberry-pi-mount-usb-drive/

MOUNTING
try:
    # Mount memory stick and open file on it
    os.system("sudo mount /dev/disk/by-uuid/0177-74FD /media/usb_logger -o noauto,users,rw,umask=0")
    self.data_file = open("/media/usb_logger/" + name + ".csv", "w+")
except:
    pass

# Also create a backup file locally in the logging folder
self.backup_file = open(os.path.dirname(os.path.abspath(__file__)) + "/logging/" + name + ".csv", "w+")

# Write the headers of each file
for file in self.data_file, self.backup_file:

UNMOUNTING
try:
    # Unmount the USB stick so that it can be safely removed
    os.system("sudo umount /media/usb_logger")
except:
    pass
'''

