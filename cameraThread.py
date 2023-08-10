import threading
import cv2

class vStream:
    def __init__(self, src):
        self.capture = cv2.VideoCapture(src)
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        while True:
            _, self.frame = self.capture.read()

    def getFrame(self):
        return self.frame