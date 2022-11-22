#!/usr/bin/env python3


import rospy

from teleop_lib.msg import RobotCommand

class PublisherPlugin:
    def __init__(self, topic="command"):
        self._pub = rospy.Publisher(topic, RobotCommand, queue_size=1)
    
    def do_command(self, cmd):
        self._pub.publish(cmd)

