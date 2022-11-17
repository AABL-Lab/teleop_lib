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

    #TODO: The profiles should change which direction affects which velocity
    of the robot's eef. 
"""

import rospy
import numpy as np
import yaml

from geometry_msgs.msg import Twist

from teleop_lib.msg import RobotCommand



class TwistAxes:
    _registry = {}
    def __init__(self, setter):
        self._name = setter.__name__
        self._setter = setter
        if self._name in TwistAxes._registry:
            raise ValueError("Cannot reuse name")
        TwistAxes._registry[self._name] = self
    def __call__(self, msg, val):
        self._setter(msg, val)
    def __repr__(self):
        return self._name

    @staticmethod
    def get(self, name):
        return TwistAxes.registry[name]

    @TwistAxes.__init__
    def AXIS_X(msg, val):
        msg.linear.x = val
    @TwistAxes.__init__
    def AXIS_Y(msg, val):
        msg.linear.y = val
    @TwistAxes.__init__
    def AXIS_Z(msg, val):
        msg.linear.z = val
    @TwistAxes.__init__
    def AXIS_ROLL(msg, val):
        msg.angular.roll = val
    @TwistAxes.__init__
    def AXIS_PITCH(msg, val):
        msg.angular.pitch = val
    @TwistAxes.__init__
    def AXIS_YAW(msg, val):
        msg.angular.yaw = val

    @staticmethod
    def axis_representer(dumper, data):
        return dumper.represent_scalar("!!str", str(data))
yaml.add_representer(TwistAxes, TwistAxes.axis_representer)


    

"""
EXAMPLE_FIXED_INPUT_MAP = {
    "axes":
        (
            { 
                "output": TwistAxes.AXIS_X,
                "scale": 1,
                "deadzone": 0.05
                },
            { 
                "output": TwistAxes.AXIS_Y,
                "scale": 1,
                "deadzone": 0.05
                },
            { 
                "output": TwistAxes.AXIS_Z,
                "scale": 1,
                "deadzone": 0.05
                }
        ),
    "buttons":
        (
            {
                "output": RobotCommand.STOP_COMMAND,
                "is_active": bool
            }
        )
}
"""

class FixedInputMap:
    def __init__(self, config):
        self._config = config

    def process_input(self, joy):
        cmd = RobotCommand()

        for ax, cfg in zip(joy.axes, self._config["axes"]):
            if "output" not in cfg:
                continue
            if np.abs(ax) < cfg.get("deadzone", 0):
                val = 0
            else:
                val = ax * cfg.get("scale", 1)
            cfg["output"](cmd.twist, ax*cfg.get("scale", 1))

        for btn, cfg in zip(joy.buttons, self._config["buttons"]):
            if "output" in cfg and cfg.get("is_active", bool)(btn):
                cmd.command = cfg["output"]

        return cmd

class ModalControlMap:
    def __init__(self, modes):
        self._modes = modes
        self._mode = 0

    def process_input(self, joy):
        cmd = self._modes[self._mode].process_input(joy)
        if cmd.command == RobotCommand.CHANGE_MODE_COMMAND:
            self._mode = (self._mode + 1) % len(self._modes)
        return cmd
