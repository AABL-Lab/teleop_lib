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
        maxvals = all_twists[imax]

        msg.twist.linear = Vector3(*maxvals[:3])
        msg.twist.angular = Vector3(*maxvals[3:])
        

        return msg



class RobotCommandSummaryFilter:
    def __init__(self, *args, **handlers):
        self._cache = message_filters.Cache(*args)
        self._policies = collections.defaultdict(LatestMsgHandler, 
                                                 {getattr(RobotCommand, k): v for k, v in handlers.items()})

    def get(self, start_time, end_time):
        msgs = self._cache.get_between(start_time, end_time)
        msg_by_type = {}
        for msg in msgs:
            if msg.command not in msg_by_type:
                msg_by_type[msg.command] = []
            msg_by_type[msg.command].append(msg)
        return {i: self._policies[i](msg_by_type[i]) for i in sorted(msg_by_type.keys())}
        
    @classmethod
    def build(cls, policy, period=0.1):
        sub = message_filters.Subscriber("joy", Joy)
        filt = InputPolicyFilter(sub, policy)
        queue_size = 100 * period * 2 # estimate 100 Hz messages and keep 2 periods worth
        filter = cls(filt, queue_size, VELOCITY_COMMAND=MaxMsgHandler())
        # keep subs alive
        filter.__connections = [sub, filt]
        return filt
