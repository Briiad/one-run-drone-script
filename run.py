# Necessary libraries
import cv2
import numpy as py
import os
import sys
import time
import argparse

# Import Arducam library
from JetsonCamera import Camera
from Focuser import Focuser

# Import Jetson Inference library
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

# Argument parser function for i2c bus and i2c address
def arg_parse():
    parser = argparse.ArgumentParser(description='Arducam Controller.')
    parser.add_argument('-i', '--i2c-bus', type=int, nargs=None, required=True,
                        help='Set i2c bus, for A02 is 6, for B01 is 7 or 8, for Jetson Xavier NX it is 9 and 10.')
    return parser.parse_args()

# Run DetectNet using CSI camera and V4L2 camera at the same time
def run_DetectNet():
    net = detectNet("ssd-mobilenet-v2", threshold=0.5)
    csi_camera = videoSource("csi://0")
    v4l2_camera = videoSource("/dev/video1")

    # display
    display = videoOutput("display://0")

    while display.IsStreaming():
        csi_img = csi_camera.Capture()
        v4l2_img = v4l2_camera.Capture()

        if csi_img or v4l2_camera is None:
            print("Unable to load image")
            continue
        
        csi_detection = net.Detect(csi_img)
        v4l2_detection = net.Detect(v4l2_img)

        display.Render(csi_img)
        display.Render(v4l2_img)
        display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

if __name__ == "__main__":
    run_DetectNet()