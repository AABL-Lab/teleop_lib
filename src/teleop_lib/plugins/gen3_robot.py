import armpy
from teleop_lib.msg import RobotCommand

class Gen3Plugin:
    def __init__(self, arm_name):
        self._robot = armpy.initialize("gen3")
    def do_command(self, cmd):
        if cmd.command == RobotCommand.STOP_COMMAND:
            self._robot.stop()
            # TODO: stop gripper
        elif cmd.command == RobotCommand.OPEN_GRIPPER:
            self._robot.open_gripper(block=False)
        elif cmd.command == RobotCommand.CLOSE_GRIPPER:
            self._robot.close_gripper(block=False)
        else:
            cmd = [
                cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z,
                cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z
            ]
            self._robot.cartesian_velocity_command(cmd, duration=0.1, block=False, radians=True)
#            self._gripper.set_vel(cmd.gripper_velocity)