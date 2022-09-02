import rospy

class CommandQueue:
    def __init__(self, profile, topic, queue_len=10):



        if msg.command == teleop_lib.msg.RobotCommand.VELOCITY_COMMAND:
            send_vel(msg.twist)
        elif msg.command == teleop_lib.msg.RobotCommand.STOP_COMMAND:
            estop()