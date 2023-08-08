import time
import os
import platform
import sys

from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil
#############################

targetAltitude=1
manualArm=False
############DRONEKIT#################
vehicle = connect('localhost:14550',baud=57600,wait_ready=True)

#Select /dev/ttyAMA0 for UART. /dev/ttyACM0 for USB

#########FUNCTIONS###########
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
      print("Waiting for vehicle to become armable.")
      time.sleep(1)
    print("Vehicle is now armable")
      
    vehicle.mode = VehicleMode("GUIDED")
              
    while vehicle.mode!='GUIDED':
      print("Waiting for drone to enter GUIDED flight mode")
      time.sleep(1)
    print("Vehicle now in GUIDED MODE. Have fun!!")

    if manualArm==False:
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
              
    vehicle.simple_takeoff(targetHeight) ##meters

    while True:
      print("Current Altitude: %d"%vehicle.location.global_relative_frame.alt)
      if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
        break
      time.sleep(1)
    print("Target altitude reached!!")

    return None

# Main program
arm_and_takeoff(targetAltitude)

# Hover for 10 seconds
# If theres no object detected for 10 seconds, the drone will land
print("Hovering for 10 seconds...")
time.sleep(10)

# Land
print("Landing...")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()

print("Completed")