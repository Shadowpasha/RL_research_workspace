
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    # "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot &&  make px4_sitl gazebo-classic_iris_rplidar",
    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
    "ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557 -p gcs_url:=tcp-l://",

    "ros2 run px4_offboard velocity_control"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(3)
