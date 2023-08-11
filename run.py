# Import Important Libraries
import time
import os
import platform
import sys
import math 

from JetsonCamera import Camera
from Focuser import Focuser

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import cv2

# Import Jetson Inference library
from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy, videoSource

# Global variables
manualArm = True

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
print("Connecting to mavlink...")
mav_connect = mavutil.mavlink_connection('localhost:14551', baud=57600)
print("Connecting to drone...")
vehicle = connect('localhost:14550', baud=57600, wait_ready=False)

# dronekit check heartbeat
print("Waiting for heartbeat...")
vehicle.wait_ready('autopilot_version')
print("Heartbeat received!")


# FUNCTIONS FOR NOGPS NAVIGATION

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):
    if yaw_angle is None:
      yaw_angle = vehicle.attitude.yaw

    msg = vehicle.message_factory.set_attitude_target_encode(
      0,
      1,                                         #target system
      1,                                         #target component
      0b00000000 if use_yaw_rate else 0b00000100,#type mask: bit 1 is LSB
      to_quaternion(roll_angle, pitch_angle, yaw_angle), #q
      0,                                         #body roll rate in radian
      0,                                         #body pitch rate in radian
      math.radians(yaw_rate),                    #body yaw rate in radian/second
      thrust                                    #thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
    send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
    start = time.time()

    while time.time() - start < duration:
      send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
      time.sleep(0.1)

    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0, 0, 0, True, thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# FUNTION FOR MISSION MOVEMENT

# Arm and takeoff
def arm_and_takeoff(targetHeight):    
    
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    # Arming will be done via the RC transmitter
    if manualArm:
      print("Manual arming mode")
      while not vehicle.armed:
        print("Vehicle not armed, waiting...")
        time.sleep(1)
      print("Vehicle armed")

    # GUIDED_NOGPS mode for indoor flight
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    print("Vehicle mode set to GUIDED_NOGPS")
              
    # Set the takeoff thrust
    thrust = DEFAULT_TAKEOFF_THRUST

    # Take off to target height
    while True:
      current_altitude = vehicle.location.global_relative_frame.alt
      print("Altitude: %f"%current_altitude)
      if current_altitude >= targetHeight*0.95:
        print("Reached target altitude")
        break
      elif current_altitude >= targetHeight*0.6:
        thrust = SMOOTH_TAKEOFF_THRUST
      set_attitude(thrust = thrust)
      time.sleep(0.2)

    return None

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
  arm_and_takeoff(40)
  # detector()


# Run main program
if __name__ == "__main__":
  main()