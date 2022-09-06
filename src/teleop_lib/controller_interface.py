#!/usr/bin/env python3
# Author: Isaac Sheidlower

"""
    High llevel class to interface with the controller. 
    This class communicates input_profile.py to get 
    mappings of controller input to cartesian commands.

"""
import rospy
import numpy as np
from input_profile import InputProfile

class ControllerInterface:
    def __init__(self, profile="default", linear_scale=1, angular_scale=1, deadzone=1, estop_index=0):
        rospy.init_node('controller_interface', anonymous=True)
        self.profile = profile
        self.input_profile = InputProfile(name=profile)
        self.rate = rospy.Rate(1000) # 10hz
        self.estop_index = estop_index

    def start(self):
        pass
    
    def stop(self):
        pass

    def run(self):
        pass

    

if __name__ == '__main__':
    try:
        controller_interface = ControllerInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass