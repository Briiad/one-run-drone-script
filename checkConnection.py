from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

vehicle = connect('localhost:14550')

# print APM version
print("Autopilot version: %s" % vehicle.version)

# get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable