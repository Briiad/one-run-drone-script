import cv2
import math
import time

from JetsonCamera import Camera
from Focuser import Focuser

# nvarguscamera
cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
cam_2 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")

focuser_cam_1 = Focuser(7)
focuser_cam_2 = Focuser(8)
focuser_cam_1.set(Focuser.OPT_FOCUS, 150)
focuser_cam_2.set(Focuser.OPT_FOCUS, 150)

while True:
    res, frame = cam_2.read()
    # resize
    frame = cv2.resize(frame, (1280, 720))

    if not res:
        print("Ignoring empty camera frame.")
        continue

    # line to draw a square in the middle of the frame, square is 100 px to the right and have the size of 200x200
    cv2.rectangle(frame, (640, 360), (840, 560), (0, 255, 0), 2)

    cv2.imshow("Camera 1", frame)

    if cv2.waitKey(1) == ord('q'):
        break


cam_2.release()
cv2.destroyAllWindows()