#!/usr/bin/env python3
# Author: Isaac Sheidlower

"""
    High llevel class to interface with the controller. 
    This class communicates input_profile.py to get 
    mappings of controller input to cartesian commands.

    #TODO: maybe just make the profile with the scales a dictionary?

"""
import rospy
from sensor_msgs.msg import Joy

import teleop_lib.input_profile

class ControllerInterface:
    def __init__(self, profile=None, plugins=()):

        self._profile = profile
        self._plugins = list(plugins)

        self._joy_sub = rospy.Subscriber("joy", Joy, self._joy_callback)
        #self.dynamic_client = dynamic_reconfigure.client.Client(InputProfiles, timeout=30, config_callback=self.dynamic_callback)

    def _joy_callback(self, joy):
        if self._profile is None:
            return

        cmd = self._profile.process_input(joy)
        for plugin in self._plugins:
            plugin.do_command(cmd)


if __name__ == '__main__':
    rospy.init_node('controller_interface', anonymous=True)
    controller_interface = ControllerInterface()
    rospy.spin()
