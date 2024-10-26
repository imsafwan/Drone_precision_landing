#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

# Global variables to track the drone's state and altitude
current_state = State()
current_altitude = 0.0

# Callback to update the drone's current state
def state_cb(state):
    global current_state
    current_state = state

# Callback to update the drone's current altitude
def altitude_cb(altitude):
    global current_altitude
    current_altitude = altitude.data

def takeoff():
    rospy.init_node('px4_takeoff_node', anonymous=True)

    # Subscriber to get the drone's current state
    rospy.Subscriber("/mavros/state", State, state_cb)

    # Subscriber to monitor the drone's altitude
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, altitude_cb)

    # Publisher for setting the drone's position
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Services for arming the drone and changing its flight mode
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # Set the loop rate
    rate = rospy.Rate(20)  # 20Hz

    # Ensure connection to PX4 SITL
    while not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected")

    # Create a target pose for takeoff
    takeoff_pose = PoseStamped()
    takeoff_pose.pose.position.x = 0
    takeoff_pose.pose.position.y = 0
    takeoff_pose.pose.position.z = 3  # Desired takeoff altitude

    # Send a few setpoints before starting
    for _ in range(100):
        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    # Set the mode to OFFBOARD
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"

    # Arm the drone
    arm_cmd = CommandBool()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        # If we're not in OFFBOARD mode, set it
        if current_state.mode != "OFFBOARD" and rospy.Time.now() - last_request > rospy.Duration(5.0):
            if set_mode_client(0, offb_set_mode.custom_mode).mode_sent:
                rospy.loginfo("OFFBOARD mode enabled")
            last_request = rospy.Time.now()

        # If the drone is not armed, arm it
        elif not current_state.armed and rospy.Time.now() - last_request > rospy.Duration(5.0):
            if arming_client(arm_cmd.value).success:
                rospy.loginfo("Drone armed")
            last_request = rospy.Time.now()

        # Publish the takeoff setpoint
        local_pos_pub.publish(takeoff_pose)

        # Monitor altitude and stop once the drone reaches the target
        if current_altitude >= 2.9:  # If we're close to the target altitude
            rospy.loginfo("Takeoff successful!")
            break

        rate.sleep()

if __name__ == '__main__':
    try:
        takeoff()
    except rospy.ROSInterruptException:
        pass
