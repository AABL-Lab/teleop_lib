#!/usr/bin/env python3
# Author: Isaac Sheidlower
"""
    File the listens to controller joystick input
    and publishes a twist/cartesian command based on
    an input profile. An "input profile" is a specification
    of how the joystick input should be mapped to the
    cartesian command. 

    
    The "profile" parameter is an integer that specifies the control mode.
    Higher level names are available through dynamic reconfigure or
    the InputProfiles.cfg file.
"""

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import dynamic_reconfigure

profile = 0
linear_scale = 1
angular_scale = 1
deadzone = .05

def update_parameters(config):
    global profile, linear_scale, angular_scale, deadzone
    profile = config["profile"]
    linear_scale = config["linear_scale"]
    angular_scale = config["angular_scale"]
    deadzone = config["deadzone"]

class InputProfile:
    def __init__(self):
        # self.name = name
        # self.linear_scale = linear_scale
        # self.angular_scale = angular_scale
        # self.deadzone = deadzone
        self.rate = 1000
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.twist_pub = rospy.Publisher("joy_command", Twist, queue_size=10)
        # Adding higher level button publisher
        self.button_pub = rospy.Publisher("joy_button", Int32MultiArray, queue_size=10)
        self.dynamic_client = dynamic_reconfigure.client.Client("teleop_lib", timeout=30, config_callback=update_parameters)

    def joy_callback(self, data):
        global profile, linear_scale, angular_scale, deadzone
        # Get the joystick input
        joy_x = data.axes[0]
        joy_y = data.axes[1]
        joy_z = data.axes[2]

        # Apply the deadzone
        if abs(joy_x) < deadzone:
            joy_x = 0
        if abs(joy_y) < deadzone:
            joy_y = 0
        if abs(joy_z) < deadzone:
            joy_z = 0

        # Scale the joystick input
        self.twist_msg.linear.x = joy_y * self.linear_scale
        self.twist_msg.linear.y = joy_x * self.linear_scale
        self.twist_msg.angular.z = joy_z * self.angular_scale

        # Publish the twist message
        self.twist_pub.publish(self.twist_msg)
        self.button_pub.publish(data.buttons)

    # def run(self):
    #     while not rospy.is_shutdown():
    #         self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('input_profile', anonymous=False)
        controller_interface = InputProfile()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass