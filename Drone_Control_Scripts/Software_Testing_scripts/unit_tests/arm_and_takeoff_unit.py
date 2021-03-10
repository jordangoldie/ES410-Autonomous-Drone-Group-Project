from Drone_event_unit import Drone  # import Drone class from Drone_event.py

Hex = Drone("127.0.0.1:14550")      # Create instance of drone class, passing IP and Port for UDP socket
print('[INFO MAIN] >> UDP connection to SITL established')

test_alt = 15                    # specify target altitude (m)
Hex.arm_and_takeoff(test_alt)    # run arm and take off function






