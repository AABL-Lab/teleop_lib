#!/usr/bin/env python3

from math import radians
from controller_interface import ControllerInterface
from gen3_testing.gen3_movement_utils import Arm
import rospy
import numpy as np


try:
    rospy.init_node('controller_testing', anonymous=True)
except:
    pass

arm = Arm()
#arm.home_arm()
controller = ControllerInterface()
while not rospy.is_shutdown():
    cmd = controller.get_user_command()
    arm.cartesian_velocity_command([cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z, cmd.angular.x, \
        cmd.angular.y], duration=.1, radians=True)
    #buttons = controller.get_user_buttons()
    #print(buttons)