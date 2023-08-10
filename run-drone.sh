# Step 1 Initialize MAVProxy on Different Terminal
echo "Initializing MAVProxy"
# sudo mavproxy.py --master="/dev/ttyTHS1" --baudrate 57600 --out="localhost:14550"

# Step 2 Run the Drone
echo "I hope you are ready to fly!"
sudo python run.py