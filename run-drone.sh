# Initialize MAVProxy
echo "Initializing MAVProxy"
mavproxy.py --master="/dev/ttyTHS1" --baudrate 57600 --out="localhost:14550"

# Step 2 Arm and takeoff
echo "Arming and taking off"
python3 arm_and_takeoff.py

# Step 3 Initialize camera and detection module
echo "Initializing camera and detection module"
python3 Initialize_Camera.py