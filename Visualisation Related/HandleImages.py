import socket, sys, cv2, pickle, struct, zlib, imutils
from PIL import Image
from io import BytesIO
import numpy as np
from TCP import TcpServer

tcp = TcpServer(1234, 'UNITY CAMERA FEED')  # create TCPServer object
tcp.bind_server_socket()  # bind to socket
tcp.listen_for_tcp()  # listen for tcp functions

data = b""  # set data variable to type 'bytes'
payload_size = struct.calcsize(">L")  # sets payload size to 4 bytes

while True:

    while len(data) < payload_size:  # runs whilst data is less than 4 bytes long
        print("Recv: {}".format(len(data)))  # prints data length, at this point 0
        data += tcp.client_socket.recv(4096)  # receives 4096 bytes, > than 4 therefore loop breaks

    print("Done Recv: {}".format(len(data)))  # prints new data length, should be 4096
    packed_msg_size = data[:payload_size]  # gets first four bytes of data as this is the length
    data = data[payload_size:]  # takes the first four bytes (protocol) out of the data
    msg_size = struct.unpack(">L", packed_msg_size)[0]  # unpacks the 4 bytes to get message size
    print("packed_msg_size: {}".format(packed_msg_size))  # prints the packed message size
    print("msg_size: {}".format(msg_size))  # prints the message size

    while len(data) < msg_size:  # runs loop until data size is greater than or equal to the msg size
        data += tcp.client_socket.recv(4096)
    frame_data = data[:msg_size]  # takes the bytes up to the size of the msg, leaving any excess
    data = data[msg_size:]  # assigns the excess to data as it is the beginning of the next msg
    print("Frame data length: {}".format(len(frame_data)))

    stream = BytesIO(frame_data)  # creates BytesIO object
    image = Image.open(stream).convert("RGB")  # convert from RGBA
    stream.close()

    frame = np.array(image)  # get np array which cv2 can handle
    frame = imutils.resize(frame, width=400)

    cv2.imshow('unity feed', frame)  # display frame
