import serial


class Arduino:
    def __init__(self, com_port, baud_rate):
        self.serial = serial.Serial(com_port, baud_rate, timeout=.1)

    def send_plant_command(self, mode):
        self.serial.write(mode)

    def receive_message(self):
        while True:
            message = self.serial.readline()
            if message:
                print(message)
