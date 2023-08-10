from JetsonCamera import Camera
from Focuser import Focuser
import cv2
import time

from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, videoSource

print("Initializing camera...")
cam_1 = cv2.VideoCapture('/dev/video0')
# cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
focuser_cam_1 = Focuser(7)
focuser_cam_1.set(Focuser.OPT_FOCUS, 150)

print("Loading network...")
net = detectNet(argv=['--model=../models/modelv2/ssd-mobilenet.onnx', '--labels=../models/modelv2/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.6)

def detector():
    while True:
        res, frame = cam_1.read()
        # frame_csi = payload_cam.getFrame() 

        if not res:
            print("Ignoring empty camera frame.")
            continue

        # Detect objects
        img = cudaFromNumpy(frame)
        # img_csi = cudaFromNumpy(frame_csi)
        detections = net.Detect(img)
        # detections_csi = net.Detect(img_csi)

        # print the detections class names and confidence
        print("detected {:d} objects in image".format(len(detections)))

        # Draw detections and coordinates of the centroid
        for detection in detections:
            # coordinates of the centroid
            x = int(detection.Center[0])
            y = int(detection.Center[1])
            # draw a crosshair on the center of the object
            cv2.line(frame, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
            cv2.line(frame, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
            # draw a bounding box around the object
            cv2.rectangle(frame, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), (255, 0, 0), 2)
            # draw the label
            cv2.putText(frame, net.GetClassDesc(detection.ClassID), (int(detection.Left), int(detection.Top - 5)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
          
            # Show FPS (0.0)
            fps_time = time.time()
            cv2.putText(frame, "FPS: " + str(int(1.0 / (time.time() - fps_time))), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Display
        cv2.imshow("Frame", frame)
         # cv2.imshow("Frame CSI", frame_csi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cam_1.release()
            # payload_cam.capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    detector()
