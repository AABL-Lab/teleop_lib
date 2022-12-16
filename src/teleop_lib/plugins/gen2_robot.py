#!/usr/bin/env python3

import armpy.gen2_teleop
import armpy.gripper

from teleop_lib.msg import RobotCommand

class Gen2Plugin:
    def __init__(self):
        self._robot = armpy.gen2_teleop.Gen2Teleop()
        self._gripper = armpy.gripper.Gripper()

    def do_command(self, cmd):
        if cmd.command == RobotCommand.STOP_COMMAND:
            self._robot.stop()
            # TODO: stop gripper
        else:
            self._robot.set_velocity(cmd.twist)

            # This still blocks for 1s even if block is False! 
            # TODO: make it not block or put it on a separate thread so it doesn't kill teleop
            # if cmd.command == RobotCommand.OPEN_GRIPPER:
            #     self._gripper.open(block=False)  
            # elif cmd.command == RobotCommand.CLOSE_GRIPPER:
            #     self._gripper.close(block=False)
            # else:
            self._gripper.set_vel(cmd.gripper_velocity)

