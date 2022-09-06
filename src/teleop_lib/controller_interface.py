#!/usr/bin/env python3
# Author: Isaac Sheidlower

"""
    High llevel class to interface with the controller. 
    This class communicates input_profile.py to get 
    mappings of controller input to cartesian commands.

"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import dynamic_reconfigure

class ControllerInterface:
    def __init__(self, profile="default", linear_scale=1, angular_scale=1, deadzone=1, estop_index=0):
        rospy.init_node('controller_interface', anonymous=True)
        self.profile = profile
        self.rate = rospy.Rate(1000) # 10hz
        self.estop_index = estop_index
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.deadzone = deadzone
        self.latest_input = Twist()
        self.input_profile_sub = rospy.Subscriber("joy_command", Twist, self.user_input_callback)
        self.dynamic_client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)

    def user_input_callback(self, data):
        self.latest_input.linear.x = data.linear.x
        self.latest_input.linear.y = data.linear.y
        self.latest_input.linear.z = data.linear.z
        self.latest_input.angular.x = data.angular.x
        self.latest_input.angular.y = data.angular.y
        self.latest_input.angular.z = data.angular.z

    def dynamic_callback(self, config):
        self.profile = config["profile"]
        self.linear_scale = config["linear_scale"]
        self.angular_scale = config["angular_scale"]
        self.deadzone = config["deadzone"]
    def update_input_profile(self, profile=None, linear_scale=None, angular_scale=None, deadzone=None):
        if profile is not None:
            self.profile = profile


        

    def get_user_input():
        return self.latest_input
    

    

if __name__ == '__main__':
    try:
        controller_interface = ControllerInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass