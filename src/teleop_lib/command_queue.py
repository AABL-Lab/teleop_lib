import collections
import copy
import numpy as np

from geometry_msgs.msg import Vector3
import message_filters
import rospy
from sensor_msgs.msg import Joy

from teleop_lib.msg import RobotCommand

class InputPolicyFilter(message_filters.SimpleFilter):
    def __init__(self, f, input_policy):
        super().__init__()
        self._policy = input_policy
        self.connectInput(f)

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.process)

    def process(self, msg):
        self.signalMessage(self._policy.process_input(msg))


class LatestMsgHandler:
    def __call__(self, msgs):
        return msgs[-1]
    
class MaxMsgHandler:
    def __call__(self, msgs):
        msg = copy.deepcopy(msgs[-1])

        all_twists = np.array([[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
                                for msg in msgs])
        imax = np.argmax(np.abs(all_twists), axis=0)
        maxvals = all_twists[imax, list(range(6))]

        msg.twist.linear = Vector3(*maxvals[:3])
        msg.twist.angular = Vector3(*maxvals[3:])
        

        return msg



class RobotCommandSummaryFilter(message_filters.Cache):
    def __init__(self, *args, **kwargs):
        self._policies = collections.defaultdict(LatestMsgHandler)
        self._policies[RobotCommand.VELOCITY_COMMAND] = MaxMsgHandler()
        
        for arg in kwargs.keys():
            if hasattr(RobotCommand, arg):
                self._policies[getattr(RobotCommand, arg)] = kwargs[arg]
                del kwargs[arg]

        super().__init__(*args, **kwargs)

    def get(self, start_time, end_time):
        msgs = self.getInterval(start_time, end_time)
        msg_by_type = {}
        for msg in msgs:
            if msg.command not in msg_by_type:
                msg_by_type[msg.command] = []
            msg_by_type[msg.command].append(msg)
        print({k: len(v) for k,v in msg_by_type.items()})
        return {i: self._policies[i](msg_by_type[i]) for i in sorted(msg_by_type.keys())}

 
