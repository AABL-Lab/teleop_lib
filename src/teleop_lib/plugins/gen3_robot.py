import armpy
from teleop_lib.msg import RobotCommand
from kortex_driver.msg import BaseCyclic_Feedback

DEFAULT_CONSTRAINTS = {
    'x': (0.3, 0.8),
    'y': (-0.25, 0.25),
    'z': (0.015, 0.6),
    'buffer_multiplier': 1.0
}

VELOCITY_CAP = 0.11

class Gen3Plugin:
    GRIPPER_INCREMENT = 0.1
    DURATION = 200
    CARTESIAN_VELOCITY_DURATION = 0.1
    def __init__(self, arm_name, constraints=None):
        self._robot = armpy.initialize(arm_name)
        self._constraints = DEFAULT_CONSTRAINTS
        if self._constraints:
            self.minx, self.maxx = self._constraints['x']
            self.miny, self.maxy = self._constraints['y']
            self.minz, self.maxz = self._constraints['z']
            self.buffer_multiplier = self._constraints['buffer_multiplier']
        else:
            print(f"No movement constraints found.")

    def do_command(self, cmd, current_state: BaseCyclic_Feedback = None):
        if cmd.command == RobotCommand.STOP_COMMAND:
            # print(cmd)
            self._robot.send_gripper_command(0, mode = 'speed', duration = self.DURATION, relative=True, block=False)
            self._robot.stop()
        elif cmd.command == RobotCommand.OPEN_GRIPPER:
            # print(cmd)
            #self._robot.open_gripper(block=False)
            #self._robot.send_gripper_command(self.GRIPPER_INCREMENT, mode='speed', duration=1, block=False)
            self._robot.send_gripper_command(self.GRIPPER_INCREMENT, mode = 'speed', duration = self.DURATION, relative=True, block=False)
        elif cmd.command == RobotCommand.CLOSE_GRIPPER:
            # print(cmd)
            # self._robot.send_gripper_command(-self.GRIPPER_INCREMENT, mode='speed', duration=1, block=False)
            # self._robot.send_gripper_command(-self.GRIPPER_INCREMENT, relative=True, block=False)
            #self._robot.close_gripper(block=False)
            #self._robot.send_gripper_command(-1*self.GRIPPER_INCREMENT, mode = 'speed', duration = 1000, relative=True, block=False)
            self._robot.send_gripper_command(-10* self.GRIPPER_INCREMENT, mode = 'speed', duration = self.DURATION, relative=True, block=False)
        elif cmd.command == RobotCommand.GO_HOME_COMMAND:
            # print(cmd)
            self._robot.home_arm()
        else:
# @staticmethod
# def basecyclicfeedback_to_state(msg: BaseCyclic_Feedback):
#     gripper_pos = msg.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position
#     tool_pose = msg.base.tool_pose_x, msg.base.tool_pose_y, msg.base.tool_pose_z, msg.base.tool_pose_theta_x, msg.base.tool_pose_theta_y, msg.base.tool_pose_theta_z 
#     tool_v = msg.base.tool_twist_linear_x, msg.base.tool_twist_linear_y, msg.base.tool_twist_linear_z, msg.base.tool_twist_angular_x, msg.base.tool_twist_angular_y, msg.base.tool_twist_angular_z
#     return np.array([*tool_pose, *tool_v, gripper_pos], dtype=np.float32)

            # If we have a current_state message. Block movement within bounds.
            vx, vy, vz, vr, vp, vyaw = cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z, cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z
            if abs(vx) > VELOCITY_CAP: vx = VELOCITY_CAP if vx > 0 else -VELOCITY_CAP
            if abs(vy) > VELOCITY_CAP: vy = VELOCITY_CAP if vy > 0 else -VELOCITY_CAP
            if abs(vz) > VELOCITY_CAP: vz = VELOCITY_CAP if vz > 0 else -VELOCITY_CAP

            if self._constraints and current_state:
                newx = current_state.base.tool_pose_x + vx * self.buffer_multiplier * self.CARTESIAN_VELOCITY_DURATION
                newy = current_state.base.tool_pose_y + vy * self.buffer_multiplier * self.CARTESIAN_VELOCITY_DURATION
                newz = current_state.base.tool_pose_z + vz * self.buffer_multiplier * self.CARTESIAN_VELOCITY_DURATION
                if (newx < self.minx and vx < 0) or (newx > self.maxx and vx > 0): vx = 0; print(f"WARN: constraining zeroing x velocity. {newx=:1.2f}")
                if (newy < self.miny and vy < 0) or (newy > self.maxy and vy > 0): vy = 0; print(f"WARN: constraining zeroing y velocity. {newy=:1.2f}")
                if (newz < self.minz and vz < 0) or (newz > self.maxz and vz > 0): print(f"WARN: constraining zeroing z velocity. {newz=:1.2f} {vz=:1.2f}"); vz = 0; 

            cmd = [vx,vy,vz,vr,vp,vyaw]
            self._robot.cartesian_velocity_command(cmd, duration=self.CARTESIAN_VELOCITY_DURATION, block=False, radians=True)
#            self._gripper.set_vel(cmd.gripper_velocity)