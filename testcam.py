import cv2

from JetsonCamera import Camera
from Focuser import Focuser

# nvarguscamera
cam_1 = cv2.VideoCapture("/dev/video0")
# cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
# cam_2 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")

# focuser_cam_1 = Focuser(7)
# focuser_cam_2 = Focuser(8)
# focuser_cam_1.set(Focuser.OPT_FOCUS, 150)
# focuser_cam_2.set(Focuser.OPT_FOCUS, 150)

while True:
    ret, frame1 = cam_1.read()
    # ret, frame2 = cam_2.read()

    frame1 = cv2.resize(frame1, (640, 480))
    # frame2 = cv2.resize(frame2, (640, 480))

    cv2.imshow("cam1", frame1)
    # cv2.imshow("cam2", frame2)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cam_1.release()
# cam_2.release()
cv2.destroyAllWindows()