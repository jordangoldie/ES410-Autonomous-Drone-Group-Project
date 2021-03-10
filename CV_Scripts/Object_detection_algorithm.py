from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2
import math
import socket
import sys

print(sys.version_info)


def truncate(n, decimals=0):
    multiplier = 10 ** decimals
    return int(n * multiplier) / multiplier


ap = argparse.ArgumentParser()
ap.add_argument('-p', '--prototxt', required=True,
                help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", required=True,
                help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.7,
                help="minimum probability to filter weak detections")
ap.add_argument("-f", "--focalLength", type=float, default=330,
                help="focal length in pixels")
ap.add_argument("-s", "--socket", required=True, type=int,
                help="socket port number")
ap.add_argument("-I", "--index", required=True, type=int,
                help="camera index")

args = vars(ap.parse_args())

# initialise the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicyle", "bird", "boat",
           "bottle", "bus", "car", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load model
print("[INFO] loading model.....")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

# initialise video stream and FPS
print("[INFO] starting video stream.....")
vs = VideoStream(src=args["index"]).start()
time.sleep(2.0)
fps = FPS().start()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('localhost', args["socket"]))
s.listen(5)
print('[INFO] Waiting for TCP connection.....')

theta = 0
fi = 0

while True:

    clientsocket, address = s.accept()
    print(f"[INFO] connection from {address} has been established")
    clientsocket.send(bytes('welcome to the server', "utf-8"))

    # loop over frame
    while True:

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break

        # grab frame from videostream and resize to max width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # grab frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                     0.007834, (300, 300), 127.5)
        # pass blob through the network and obtain detections and predictions
        net.setInput(blob)
        detections = net.forward()

        # loop over detections
        for i in np.arange(0, detections.shape[2]):
            # extract confidence associated with prediction
            confidence = detections[0, 0, i, 2]

            # filter out weak detections
            if confidence > args["confidence"]:
                # extract index of class label from 'detections'
                # compute (x, y) coords of bounding box
                idx = int(detections[0, 0, i, 1])
                if idx == 15:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # draw the prediction on the frame
                    # label = "{}: {:.2f}%".format(CLASSES[idx-1], confidence*100)
                    # label = f'{CLASSES[idx - 1]} Detection{i}'
                    label = 'person'
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[5], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[5], 2)

                    # compute detection centre, and box-corner coordinates
                    DCx = (startX + endX) / 2
                    DCy = (startY + endY) / 2

                    # draw centre point of detection box
                    cv2.circle(frame, (int(DCx), int(DCy)), 1, COLORS[5], 2)

                    # calculate angles relative to principle axis
                    xa = DCx - w / 2
                    theta = (math.atan(xa / args["focalLength"]))

                    # theta = truncate(theta, 3)
                    ya = DCy - h / 2
                    fi = (math.atan(ya / args["focalLength"]))
                    fi = truncate(fi, 3)

                    # print info to terminal
                    print(f'[INFO] theta, fi : {theta},{fi}')
                    print(DCx)

                    # send data to client server
                    detect = 1
                    clientsocket.send(bytes(f'{theta},{fi},{detect}', "utf-8"))

            '''else:
                detect = 0
                clientsocket.send(bytes(f'{detect}', "utf-8"))'''

        # show the output frame
        cv2.imshow("Frame", frame)

        # update the FPS counter
        fps.update()

    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx FPS: {:.2f}".format(fps.fps()))

    cv2.destroyAllWindows()
    vs.stop()
    # clientsocket.close()
    break


