#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State, ExtendedState
from std_msgs.msg import Bool
from simple_pid import PID
from cv2 import aruco

class ArUcoPrecisionLanding:
    def __init__(self):
        rospy.init_node('aruco_precision_landing_node', anonymous=False)

        # ArUco marker detection setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)  # Fix here
        self.aruco_params = aruco.DetectorParameters_create()
        self.marker_length = 2  # Marker size in meters

        # Camera setup
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/iris_customized/usb_cam/image_raw", Image, self.image_callback)
        self.landed = False  # Track if the drone has landed
        self.extended_state_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extended_state_callback)

        # MAVROS services and publishers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.alt_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.altitude_callback)

        # Publisher for stop signal
        self.stop_setpoints_pub = rospy.Publisher("/stop_setpoints", Bool, queue_size=10)

        # Flight control variables
        self.current_state = None
        self.is_marker_detected = False
        self.marker_position = None
        self.is_offboard_mode_set = False
        self.current_altitude = None  # Store current altitude
        self.target_altitude = 5.0  # Target altitude in meters

        # PD controllers for smooth and precise movement
        self.xPD = PID(0.0075, 0.0, 0.0001, output_limits=(-0.5, 0.5), setpoint=0.0)
        self.yPD = PID(0.0075, 0.0, 0.0001, output_limits=(-0.5, 0.5), setpoint=0.0)
        self.zPD = PID(0.5, 0.0, 0.05, output_limits=(-1.0, 1.0), setpoint=self.target_altitude)

        self.landing_threshold = 0.1
        self.rate = rospy.Rate(20)

        

    def state_callback(self, data):
        self.current_state = data

    def extended_state_callback(self, msg):
        if msg.landed_state == 1:
            self.landed = True
        else:
            self.landed = False

    def altitude_callback(self, data):
        self.current_altitude = data.pose.position.z

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Detect ArUco markers
            corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None:
                self.is_marker_detected = True
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, np.eye(3), np.zeros(5))
                aruco.drawDetectedMarkers(cv_image, corners, ids)

                # Get the center of the detected marker
                marker_center = np.mean(corners[0][0], axis=0)
                self.marker_position = (int(marker_center[0]), int(marker_center[1]))

            else:
                self.is_marker_detected = False

            # Display the video
            cv2.imshow("ArUco Detection", cv_image)
            cv2.waitKey(3)

        except Exception as e:
            rospy.logerr("Error processing image: %s", e)

    def set_offboard_mode(self):
        for _ in range(10):
            vel_msg = TwistStamped()
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            vel_msg.twist.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        if self.current_state is not None and self.current_state.mode != 'OFFBOARD':
            self.set_mode_client(0, 'OFFBOARD')
            self.is_offboard_mode_set = True

    def align_to_marker(self):
        """ Aligns the drone to the detected ArUco marker using PD control (X, Y, and Z) """
        if self.marker_position is None or self.current_altitude is None:
            return

        # Get the image center (adjust according to camera resolution)
        image_center_x = 640 / 4  # Assuming image resolution is 640x480
        image_center_y = 480 / 4

        # Calculate the error between the marker and the image center
        error_x = self.marker_position[0] - image_center_x
        error_y = self.marker_position[1] - image_center_y

        # PD control to align the drone in X and Y directions
        control_x = self.xPD(error_y)
        control_y = self.yPD(error_x)

        # PD control for altitude (Z-axis)
        control_z = self.zPD(self.current_altitude)

        #print('Error_x', abs(error_x), 'Error_', abs(error_y), 'Error_z', abs(self.current_altitude - self.target_altitude))

        # If the altitude is within the threshold, switch to landing mode
        if abs(self.current_altitude - self.target_altitude) < 0.2 and abs(error_x) < 8 and abs(error_y) < 8:
            rospy.loginfo("Marker centered and at target altitude, preparing to land")
            self.land()
            return

        # Publish velocity commands to adjust the drone's position (X, Y, and Z)
        vel_msg = TwistStamped()
        vel_msg.twist.linear.x = control_x  # Inverting due to image frame difference
        vel_msg.twist.linear.y = control_y
        vel_msg.twist.linear.z = control_z  # Adjust altitude
        vel_msg.twist.angular.z = 0        # No yaw movement needed

        

        self.vel_pub.publish(vel_msg)

    def land(self):
        self.set_mode_client(0, 'AUTO.LAND')

    def notify_stop_setpoints(self):
        # Wait for subscribers before publishing
        while self.stop_setpoints_pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to /stop_setpoints...")
            self.rate.sleep()

        # Notify the first script to stop sending setpoints
        rospy.loginfo("Notifying to stop setpoints...")
        self.stop_setpoints_pub.publish(Bool(data=True))
        rospy.loginfo("Notified. Taking over control for landing.")

    def perform_landing(self):
        rospy.loginfo("Starting precision landing process")

        # Notify the first script to stop sending setpoints
        self.notify_stop_setpoints()

        # Set offboard mode
        self.set_offboard_mode()

        while not rospy.is_shutdown():
            if self.is_marker_detected:
                self.align_to_marker()

            if self.landed:
                rospy.loginfo("Landing complete. Ready for future commands.")
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        landing = ArUcoPrecisionLanding()
        landing.perform_landing()
    except rospy.ROSInterruptException:
        pass
