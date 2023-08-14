import cv2
import math
import time

from JetsonCamera import Camera
from Focuser import Focuser

from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy

# nvarguscamera
cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
cam_2 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")

focuser_cam_1 = Focuser(7)
focuser_cam_2 = Focuser(8)
focuser_cam_1.set(Focuser.OPT_FOCUS, 150)
focuser_cam_2.set(Focuser.OPT_FOCUS, 150)

print("Loading network...")
net = detectNet(argv=['--model=../models/modelv3/ssd-mobilenet.onnx', '--labels=../models/modelv3/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.5)

net.SetTrackingEnabled(True)
net.SetTrackingParams(minFrames=3, dropFrames=15, overlapThreshold=0.5)



while True:
    res, frame = cam_1.read()

    if not res:
        print("Ignoring empty camera frame.")
        continue

    # Detect objects
    img = cudaFromNumpy(frame)
    detections = net.Detect(img)

    # line to draw in the middle of the frame ( +10 and -10 to make sure it's in the middle )
    cv2.line(frame, (int(frame.shape[1] / 2) - 200, 0), (int(frame.shape[1] / 2) - 200, frame.shape[0]), (0, 255, 0), 2)
    cv2.line(frame, (int(frame.shape[1] / 2) + 200, 0), (int(frame.shape[1] / 2) + 200, frame.shape[0]), (0, 255, 0), 2)

    for detection in detections:
        # centroid coordinates
        x = int(detection.Center[0])
        y = int(detection.Center[1])
        # draw circle on centroid
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        # draw bounding box
        cv2.rectangle(frame, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), (255, 0, 0), 2)
        # object class name and confidence
        cv2.putText(frame, "%s (%.1f%%)" % (detection.ClassID, detection.Confidence * 100), (int(detection.Left), int(detection.Top) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        # distance between centroid and middle of the frame
        distance = math.sqrt((detection.Center[0] - int(frame.shape[1] / 2))**2 + (detection.Center[1] - int(frame.shape[0] / 2))**2)
        # draw distance
        cv2.putText(frame, "Distance: %.1f" % distance, (int(detection.Left), int(detection.Top) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        print("Distance: %.1f" % distance)
        # if centroid coordinates is in the middlle of the line, drop payload
        if x >= int(frame.shape[1] / 2) + 200 and x <= int(frame.shape[1] / 2) - 200:
            # move forward a bit
            print("Moving forward a bit")
            time.sleep(1)
            # if distance is less than 200, drop payload
            if distance <= 200:
                # hover for 1 second
                print("Hovering for 1 second")
                print("Payload dropped")  
                break
            # if centroid outside the line, adjust position
        else:
            # if centroid coordinates is in the right side of the frame, roll right
            if x >= int(frame.shape[1] / 2) + 200:
                print("Roll right")
            # if centroid coordinates is in the left side of the frame, roll left
            elif x <= int(frame.shape[1] / 2) - 200:
                print("Roll left")

    cv2.imshow("Camera 1", frame)

    if cv2.waitKey(1) == ord('q'):
        print("Quitting...")
        break


cam_1.release()
cv2.destroyAllWindows()