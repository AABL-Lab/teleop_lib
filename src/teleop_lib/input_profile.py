#!/usr/bin/env python3
# Author: Isaac Sheidlower
"""
    File the listens to controller joystick input
    and publishes a twist/cartesian command based on
    an input profile. An "input profile" is a specification
    of how the joystick input should be mapped to the
    cartesian command. 

    #TODO: Add a way to switch between input profiles via
    dynamic reconfigure.
"""

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray

class InputProfile:
    def __init__(self, name, linear_scale, angular_scale, deadzone):
        self.name = name
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.deadzone = deadzone
        self.rate = 1000
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.twist_pub = rospy.Publisher("joy_commanf", Twist, queue_size=10)
        # Adding higher level button publisher
        self.button_pub = rospy.Publisher("joy_button", Int32MultiArray, queue_size=10)

    def joy_callback(self, data):
        # Get the joystick input
        joy_x = data.axes[0]
        joy_y = data.axes[1]
        joy_z = data.axes[2]

        # Apply the deadzone
        if abs(joy_x) < self.deadzone:
            joy_x = 0
        if abs(joy_y) < self.deadzone:
            joy_y = 0
        if abs(joy_z) < self.deadzone:
            joy_z = 0

        # Scale the joystick input
        self.twist_msg.linear.x = joy_y * self.linear_scale
        self.twist_msg.linear.y = joy_x * self.linear_scale
        self.twist_msg.angular.z = joy_z * self.angular_scale

        # Publish the twist message
        self.twist_pub.publish(self.twist_msg)
        self.button_pub.publish(data.buttons)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
