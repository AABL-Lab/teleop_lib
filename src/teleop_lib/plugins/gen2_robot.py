#!/usr/bin/env python3

import rospy

import armpy.gen2_teleop

from teleop_lib.msg import RobotCommand

class Gen2Plugin:
    def __init__(self):
        self._robot = armpy.gen2_teleop.Gen2Teleop()

    def do_command(self, cmd):
        if cmd.command == RobotCommand.STOP_COMMAND:
            self._robot.stop()
        else:
            self._robot.send_velocity(cmd.twist)

