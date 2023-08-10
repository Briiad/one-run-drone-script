from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

import time

vehicle = connect('localhost:14550', baud=57600, wait_ready=True)

manualArm = False

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

if __name__ == "__main__":
    arm_and_takeoff(10)
    vehicle.close()
    print("done")