#!/usr/bin/env python3
# Author: Isaac Sheidlower

"""
    High llevel class to interface with the controller. 
    This class communicates input_profile.py to get 
    mappings of controller input to cartesian commands.

    #TODO: maybe just make the profile with the scales a dictionary?

"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client
#from teleop_lib.cfg import InputProfiles

class ControllerInterface:
    def __init__(self, profile="default", linear_scale=1, angular_scale=1, deadzone=1, estop_index=0,
        num_buttons=11):
        try:
            rospy.init_node('controller_interface', anonymous=True)
        except:
            pass
        self.profile = profile
        self.rate = rospy.Rate(1000) # 10hz
        self.estop_index = estop_index
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.deadzone = deadzone
        self.latest_commmad = Twist()
        self.num_buttons = num_buttons
        self.latest_buttons = np.array(np.zeros(self.num_buttons))
        self.input_profile_sub = rospy.Subscriber("joy_command", Twist, self.user_input_callback) # subscriber for joustick input
        self.button_sub = rospy.Subscriber("joy_button", Joy, self.user_button_callback)
        #self.dynamic_client = dynamic_reconfigure.client.Client(InputProfiles, timeout=30, config_callback=self.dynamic_callback)

    def user_input_callback(self, data):
        self.latest_commmad.linear.x = data.linear.x
        self.latest_commmad.linear.y = data.linear.y
        self.latest_commmad.linear.z = data.linear.z
        self.latest_commmad.angular.x = data.angular.x
        self.latest_commmad.angular.y = data.angular.y
        self.latest_commmad.angular.z = data.angular.z

    def user_button_callback(self, data):
        print(data)
        self.latest_buttons = np.array(data.data)

    def dynamic_callback(self, config):
        self.profile = config["profile"]
        self.linear_scale = config["linear_scale"]
        self.angular_scale = config["angular_scale"]
        self.deadzone = config["deadzone"]

    def update_input_profile(self, profile=None, linear_scale=None, angular_scale=None, deadzone=None):
        """
            Function to update any part of the input profile.
        """
        if profile is not None:
            self.profile = profile
        if linear_scale is not None:
            self.linear_scale = linear_scale
        if angular_scale is not None:
            self.angular_scale = angular_scale
        if deadzone is not None:
            self.deadzone = deadzone

        self.dynamic_callback.update_configuration(({"profile":self.profile, "linear_scale":self.linear_scale, \
            "angular_scale":self.angular_scale, "deadzone":self.deadzone}))        

    def get_user_command(self):
        """
            Function to get the user command from the controller augmented by the input profile.
        """
        return self.latest_commmad

    def get_user_buttons(self):
        """
            Function to get the user button state.
        """
        return self.latest_buttons
    
    def get_user_input(self):
        """
            Function to get the user input from the controller.
            This function returns a tuple of the cartesian command
            and the button state. 
        """
        return self.latest_commmad, self.latest_buttons

    def get_profile(self):
        return self.profile
    
    def get_linear_scale(self):
        return self.linear_scale
    
    def get_angular_scale(self):
        return self.angular_scale
    
    def get_deadzone(self):
        return self.deadzone

if __name__ == '__main__':
    try:
        controller_interface = ControllerInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass