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

import copy
import numpy as np
import yaml

from teleop_lib.msg import RobotCommand


class TwistAxis:
    __registry = {}
    def __init__(self, setter):
        self._name = setter.__name__
        self._setter = setter
        if hasattr(type(self), self._name):
            raise ValueError("cannot repeat name")
        setattr(type(self), self._name, self)
        self.__class__.__registry[self._name] = self

    def __call__(self, msg, val):
        self._setter(msg, val)
    def __repr__(self):
        return self._name

    @classmethod
    def get(cls, name):
        return cls.__registry[name]
    @classmethod
    def list(cls):
        return list(cls.__registry.keys())

    @staticmethod
    def axis_representer(dumper, data):
        return dumper.represent_str(str(data))
        
yaml.add_representer(TwistAxis, TwistAxis.axis_representer)

@TwistAxis
def AXIS_X(msg, val):
    msg.twist.linear.x = val
@TwistAxis
def AXIS_Y(msg, val):
    msg.twist.linear.y = val
@TwistAxis
def AXIS_Z(msg, val):
    msg.twist.linear.z = val
@TwistAxis
def AXIS_ROLL(msg, val):
    msg.twist.angular.x = val
@TwistAxis
def AXIS_PITCH(msg, val):
    msg.twist.angular.y = val
@TwistAxis
def AXIS_YAW(msg, val):
    msg.twist.angular.z = val
@TwistAxis
def AXIS_GRIPPER(msg, val):
    msg.gripper_velocity = val

    

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
                "output": RobotCommand.STOP_COMMAND
            }
        )
}
"""

class FixedInputProfile:
    def __init__(self, config):
        self._config = config

    def process_input(self, joy):
        cmd = RobotCommand(header=joy.header)

        for ax, cfg in zip(joy.axes, self._config["axes"]):
            if "output" not in cfg or cfg["output"] is None:
                continue
            if np.abs(ax) < cfg.get("deadzone", 0):
                val = 0
            else:
                val = ax * cfg.get("scale", 1)
            cfg["output"](cmd, val)

        for btn, cfg in zip(joy.buttons, self._config["buttons"]):
            if "output" not in cfg or cfg["output"] is None:
                continue
            if btn:
                cmd.command = cfg["output"]

        return cmd

class ModalControlMap:
    def __init__(self, modes):
        self._modes = modes
        self._mode = 0
        self._mode_changing = False

    def process_input(self, joy):
        cmd = self._modes[self._mode].process_input(joy)

        if cmd.command == RobotCommand.CHANGE_MODE_COMMAND:
            if not self._mode_changing:
                self._mode = (self._mode + 1) % len(self._modes)
            self._mode_changing = True
        else:
            self._mode_changing = False
        return cmd

def build_profile(config):
    new_config = copy.deepcopy(config)
    if "axes" not in config:
        new_config["axes"] = []
    if "buttons" not in config:
        new_config["buttons"] = []

    for ax in new_config["axes"]:
        output = ax.get("output", None)
        if isinstance(output, TwistAxis) or output is None:
            continue
        try:
            axis = TwistAxis.get(output)
        except KeyError:
            axis = None
            pass
        ax["output"] = axis

    for btn in new_config["buttons"]:
        output = btn.get("output", None)
        if output is None:
            continue
        try:
            btn["output"] = getattr(RobotCommand, output)
        except AttributeError:
            btn["output"] = None

    return FixedInputProfile(new_config)



if __name__ == "__main__":
    import sensor_msgs.msg
    joy = sensor_msgs.msg.Joy(axes=[0.3, -3], buttons=[1, 0])
    with open("../../data/XYMode.yaml") as f:
        cfg = yaml.safe_load(f)
    mode = build_profile(cfg)
    print("----")
    print(mode._config)
    print(joy)
    print(mode.process_input(joy))
