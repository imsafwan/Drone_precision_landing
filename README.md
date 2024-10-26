# Drone Precision Landing

## Overview
This package performs vision-based autonomous landing of a drone on an ArUco marker using PD control.

## Installation

### 1. Dependencies
Ensure the following dependencies are installed before setting up the package:

- **ROS Noetic (or other versions)**:
    ```bash
    sudo apt-get install ros-noetic-desktop-full
    ```

- **PX4-Autopilot**:
    Clone the PX4-Autopilot repository into your home directory:
    ```bash
    git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
    ```

### 2. Clone this Repository
Clone this repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone <repository-url> Drone_precision_landing_Gazebo
```



## 3. Setting Up the Environment

- **Source your ROS environment**:
    ```bash
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    ```

- **Add the custom drone model to PX4**:
    Copy the custom drone SDF file from the `models` folder of this repository to the PX4 directory:
    ```bash
    cp ~/catkin_ws/src/Drone_precision_landing_Gazebo/models/iris_customized.sdf ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
    ```

- **Build the catkin workspace**:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

## 4. How to Run the Simulation

Follow these steps to run the simulation:

- **Launch the UAV in Gazebo**:
    ```bash
    cd ~/catkin_ws
    source ~/catkin_ws/devel/setup.bash
    roslaunch Drone_precision_landing_Gazebo run_sim.launch
    ```

- **Launch the UAV for takeoff**:
    ```bash
    cd ~/catkin_ws/src/Drone_precision_landing_Gazebo/Scripts
    python3 drone_takeoff.py
    ```

- **Launch the landing action for the UAV**:
    ```bash
    cd ~/catkin_ws/src/Drone_precision_landing_Gazebo/Scripts
    python3 drone_land.py
    ```

