from vision import DroneCamVision
import socket, sys, cv2, pickle, struct, zlib, imutils
from PIL import Image
from io import BytesIO
import numpy as np

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
IP = input('Input IP >> ')
client_socket.connect((IP, 5311))  # here we give the IP and port we WANT to connect to

vision = DroneCamVision(1234)  # creates instance of vision.py DroneCamVision class with TCP to unity
vision.model_setup() # loads the neural networks etc.

data = b""
payload_size = struct.calcsize(">L")

vision.tcp.send_message('1')
msg = 0

while True:

    # press 'q' key to break loop
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break

    while len(data) < payload_size:  # runs whilst data is less than 4
        # print("Recv: {}".format(len(data)))  # prints data length, at this point 0
        data += vision.tcp.client_socket.recv(4096)  # receives 4096 bytes, > than 4 therefore loop breaks

    # print("Done Recv: {}".format(len(data)))  # prints new data length, should be 4096
    packed_msg_size = data[:payload_size]  # gets first four bytes of data as this is the length
    data = data[payload_size:]  # takes the first four bytes (protocol) out of the data
    msg_size = struct.unpack(">L", packed_msg_size)[0]  # unpacks the 4 bytes to get message size
    # print("packed_msg_size: {}".format(packed_msg_size))  # prints the packed message size
    # print("msg_size: {}".format(msg_size))  # prints the message size

    while len(data) < msg_size:  # runs loop until data size is greater than or equal to the msg size
        data += vision.tcp.client_socket.recv(4096)
    frame_data = data[:msg_size]  # takes the bytes up to the size of the msg, leaving any excess
    data = data[msg_size:]  # assigns the excess to data as it is the beginning of the next msg
    # print("Frame data length: {}".format(len(frame_data)))
    # print(frame_data)

    stream = BytesIO(frame_data)
    image = Image.open(stream).convert("RGB")  # changed from RGBA
    stream.close()

    frame = np.array(image)
    frame = imutils.resize(frame, width=400)

    # grab frame dimensions and convert it to a blob
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 0.007834, (300, 300), 127.5)
    # pass blob through the network and obtain detections and predictions
    vision.net.setInput(blob)
    detections = vision.net.forward()
    vision.detect = 0

    # loop over detections
    for i in np.arange(0, detections.shape[2]):
        # extract confidence associated with prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections
        if confidence > 0.8:
            # extract index of class label from 'detections'
            # compute (x, y) coordinates of bounding box
            idx = int(detections[0, 0, i, 1])
            if idx == 15:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # draw the prediction on the frame
                label = f'person {confidence * 100}'
                cv2.rectangle(frame, (startX, startY), (endX, endY), vision.colours[5], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, vision.colours[5], 2)
                vision.detect = 1

    client_socket.send(bytes(f'{vision.detect}', "utf-8"))
    print(vision.detect)