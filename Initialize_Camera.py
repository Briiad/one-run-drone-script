# Import Jetson Inference library
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

# Import OpenCV library
import cv2

# Global
net = detectNet(argv['model=../models/modelv1/ssd-mobilenet.onnx', '--labels=../models/modelv1/labels.txt', '--input-blob=input_0', 'output-cvg=scores', 'output-bbox=boxes'], threshold=0.5)

# Initialize camera
webcam = cv2.VideoCapture(0)
csicam = cv2.VideoCapture("csi://0")

# function for run the detection
def run_detection():
    while True:
        # read the current frame
        _, frame = webcam.read()
        # detect objects in the current frame
        detections = net.Detect(frame)

        cv2.imshow("frame", frame)
        # press q to close the frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # release the VideoCapture object
    webcam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    