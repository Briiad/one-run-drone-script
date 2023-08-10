# Import Important Libraries
import time
import os
import platform
import sys

from JetsonCamera import Camera
from Focuser import Focuser

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import pymavlink
import cv2

from cameraThread import vStream

# Import Jetson Inference library
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, videoSource

# Global variables
manualArm = False

# Network
print("Loading network...")
net = detectNet(argv=['--model=../models/modelv1/ssd-mobilenet.onnx', '--labels=../models/modelv1/labels.txt', '--input-blob=input_0', '--output-cvg=scores', '--output-bbox=boxes'], threshold=0.5)

# Camera
print("Initializing camera...")
cam_1 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")
cam_2 = cv2.VideoCapture("gst-launch-1.0 nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink")

focuser_cam_1 = Focuser(7)
focuser_cam_1.set(Focuser.OPT_FOCUS, 150)
focuser_cam_2 = Focuser(8)
focuser_cam_2.set(Focuser.OPT_FOCUS, 150)

# Drone connection
print("Connecting to drone...")
vehicle = connect('localhost:14550', baud=57600, wait_ready=True)

# ALL FUNCTIONS

# Arm and takeoff
def arm_and_takeoff(targetHeight):    

    # Disable prearm check
    vehicle.parameters['ARMING_CHECK']=1

    if manualArm == False:
      vehicle.armed = True
      while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    else:
      if vehicle.armed == False:
        print("Exiting script. manualArm set to True but vehicle not armed.")
        print("Set manualArm to True if desiring script to arm the drone.")
        return None
      print("Look out! Props are spinning!!")

    vehicle.mode = VehicleMode("GUIDED")
              
    while vehicle.mode!='GUIDED':
      print("Waiting for drone to enter GUIDED flight mode")
      time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")
              
    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
      print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
      if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
        break
      time.sleep(1)
    print("Target altitude reached!!")

    return None

# Drone Velocity Movement
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
      0,
      0, 0,
      pymavlink.mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
      0b0000111111000111,
      0, 0, 0,
      velocity_x, velocity_y, velocity_z,
      0, 0, 0,
      0, 0)
    
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
      vehicle.send_mavlink(msg)
      time.sleep(1)

# Drone Yaw Movement
def condition_yaw(heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    msg = vehicle.message_factory.command_long_encode(
      0, 0, # target system, target component
      pymavlink.mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
      0, #confirmation
      heading, # param 1, yaw in degrees
      0, # param 2, yaw speed deg/s
      1, # param 3, direction -1 ccw, 1 cw
      is_relative, # param 4, relative offset 1, absolute angle 0
      0, 0, 0) # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)

# Obstacle avoidance using LIDAR

# Payload pickup

# Payload drop

# Landing

# Detection
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

        # Draw detections and coordinates of the centroid
        for detection in detections:
          # coordinates of the centroid
          x = int(detection.Center[0])
          y = int(detection.Center[1])
          # draw a circle on the centroid
          cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
          # draw a rectangle on the object
          cv2.rectangle(frame, (int(detection.Left), int(detection.Top)), (int(detection.Right), int(detection.Bottom)), (255, 0, 0), 2)
          
          # for detection_csi in detections_csi:
          #   # coordinates of the centroid
          #   x_csi = int(detection_csi.Center[0])
          #   y_csi = int(detection_csi.Center[1])
          #   # draw a circle on the centroid
          #   cv2.circle(frame_csi, (x_csi, y_csi), 5, (0, 0, 255), -1)
          #   # draw a rectangle on the object
          #   cv2.rectangle(frame_csi, (int(detection_csi.Left), int(detection_csi.Top)), (int(detection_csi.Right), int(detection_csi.Bottom)), (255, 0, 0), 2)
          
          # Show FPS
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

# Payload detection using bottom camera

# Main program
def main():
  print("Starting program")
  arm_and_takeoff(1)
  # detector()


# Run main program
if __name__ == "__main__":
  main()