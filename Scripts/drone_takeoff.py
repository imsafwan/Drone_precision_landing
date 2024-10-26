#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool

current_state = State()
current_altitude = 0.0
stop_setpoints = False  # Flag to stop sending setpoints

def state_cb(state):
    global current_state
    current_state = state

def altitude_cb(altitude):
    global current_altitude
    current_altitude = altitude.data

# Callback to stop sending setpoints when landing script is active
def stop_setpoints_cb(msg):
    global stop_setpoints
    stop_setpoints = msg.data

def takeoff_and_hold():
    rospy.init_node('px4_takeoff_node', anonymous=True)

    # Subscribers for drone state, altitude, and stop setpoints signal
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, altitude_cb)
    rospy.Subscriber("/stop_setpoints", Bool, stop_setpoints_cb)

    # Publisher for local position setpoints
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Services for arming and setting mode
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20)  # 20Hz

    # Wait for FCU connection
    while not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected")

    # Takeoff setpoint
    takeoff_pose = PoseStamped()
    takeoff_pose.pose.position.x = 0
    takeoff_pose.pose.position.y = 0
    takeoff_pose.pose.position.z = 15  # Desired takeoff altitude

    # Send a few setpoints before starting
    for _ in range(2):
        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    # Arm the drone and set mode to OFFBOARD
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown() and not stop_setpoints:
        if current_state.mode != "OFFBOARD" and rospy.Time.now() - last_request > rospy.Duration(2.0):
            if set_mode_client(0, offb_set_mode.custom_mode).mode_sent:
                rospy.loginfo("OFFBOARD mode enabled")
            last_request = rospy.Time.now()

        elif not current_state.armed and rospy.Time.now() - last_request > rospy.Duration(5.0):
            if arming_client(arm_cmd.value).success:
                rospy.loginfo("Drone armed")
            last_request = rospy.Time.now()

        local_pos_pub.publish(takeoff_pose)

        if current_altitude >= 14.9:  # Target altitude
            rospy.loginfo("Holding position at target altitude.")
            break

        rate.sleep()

    

    # Maintain position until stop_setpoints is received
    rospy.loginfo("Holding position until landing script takes over...")
    while not rospy.is_shutdown() and not stop_setpoints:
        local_pos_pub.publish(takeoff_pose)
        rate.sleep()

    rospy.loginfo("Stopping setpoints. Landing script should take control now.")

if __name__ == '__main__':
    try:
        takeoff_and_hold()
    except rospy.ROSInterruptException:
        pass
