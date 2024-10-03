#!/bin/bash

# Export the desired GPS coordinates for home location
export PX4_HOME_LAT=37.413229
export PX4_HOME_LON=-121.997080
export PX4_HOME_ALT=0.0

# Set the PX4 Firmware directory
FIRMDIR=${1:-"/home/safwan/PX4-Autopilot"} # Use default firmware directory if no argument is provided
echo "Using PX4 Firmware directory: $FIRMDIR"

# Check if the firmware directory exists
if [ ! -d "$FIRMDIR" ]; then
  echo "Error: PX4 Firmware directory does not exist: $FIRMDIR"
  exit 1
fi

# Source the ROS setup for your ROS installation
source /opt/ros/noetic/setup.bash  # Ensure this matches your ROS version (e.g., noetic, melodic)

# Source the ROS workspace for your packages (e.g., catkin_ws)
source /home/safwan/catkin_ws/devel/setup.bash  # Ensure this is the correct path to your workspace

# Navigate to the PX4 Firmware directory
cd $FIRMDIR || { echo "Failed to change directory to $FIRMDIR"; exit 1; }

# Setup PX4 SITL environment (verify setup_gazebo.bash exists)
if [ -f "Tools/setup_gazebo.bash" ]; then
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
else
    echo "Error: setup_gazebo.bash not found in $FIRMDIR/Tools"
    exit 1
fi

# Update ROS_PACKAGE_PATH to include PX4 and sitl_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

# Print the current ROS_PACKAGE_PATH to verify the environment
echo "ROS_PACKAGE_PATH is set to: $ROS_PACKAGE_PATH"

# Launch the drone in PX4 SITL using roslaunch
roslaunch Drone_precision_landing_Gazebo drone_launch.launch



sleep 5 
