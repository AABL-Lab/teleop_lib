#!/usr/bin/env python3

import asyncio
import rospy

import sensor_msgs.msg

import study_runner
from study_runner.frames.logging import LoggingFrame, RunLogging
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
import tkinter

# TODO: do something smarter with this
class AsyncSubscriber:
    def __init__(self, topic, type, queue_size=0):
        self._subscriber = rospy.Subscriber(topic, type, self._cb)
        self._queue = asyncio.Queue(queue_size)
    
    def _cb(self, msg):
        try:
            self._queue.put_nowait(msg)
        except asyncio.QueueFull:
            rospy.logwarn_throttle(5, f"Queue for subscriber {self._subscriber.get_topic()} full!")

    def get(self):
        return self._queue.get()

async def run_teleop(config, status_cb):
    profile, plugin = get_teleop_info(config)
    sub = AsyncSubscriber("/joy", sensor_msgs.msg.Joy)
    with RunLogging:
        while not rospy.is_shutdown():
            msg = await sub.get()
            cmd = profile.process_input(msg)
            plugin.do_command(cmd)


def main():
    rospy.init_node("collect_teleop_data", anonymous=True)

    root = tkinter.Tk()
    runner = study_runner.StudyRunner(root, run_teleop)
    runner.add_config_frame(TeleopConfigFrame, "Teleoperation")
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()

