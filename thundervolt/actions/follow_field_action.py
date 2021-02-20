import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.utils import from_polar, assert_angle
from thundervolt.core.data import FieldData, ROBOT_SIZE
from thundervolt.core.command import RobotCommand

class FollowFieldAction(Action):
    def __init__(self, kp_lin, ki_lin, kd_lin, tolerance_lin, kp_ang, ki_ang, kd_ang, tolerance_ang):
        super().__init__()
        self.tolerance_lin = tolerance_lin
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin)
        self.tolerance_ang = tolerance_ang
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang)
        self.controller_ang.saturation = np.pi * 2 * kp_ang

    def initialize(self, robot_id, vector_field):
        super().initialize(robot_id)
        self.controller_ang.reset()
        self.controller_lin.reset()
        self.vector_field = vector_field
        self.controller_lin.set_point = 0

    def set_goal(self, goal):
        self.goal = goal

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        actual_angle = field_data.robots[self.robot_id].position.theta
        looking_direction = from_polar(actual_angle)

        pos_x = field_data.robots[self.robot_id].position.x
        pos_y = field_data.robots[self.robot_id].position.y
        robot_center = np.array((pos_x, pos_y))

        goal_vector = self.goal - robot_center

        front_direction = robot_center + (looking_direction * ROBOT_SIZE / 2)
        univector = self.vector_field.compute(front_direction)
        ang_univector = np.arctan2(univector[1], univector[0])

        ang_univector = np.arctan2(goal_vector[1], goal_vector[0])
        response_lin = -self.controller_lin.update(np.linalg.norm(goal_vector))

        if abs(assert_angle(ang_univector - actual_angle)) > np.pi/2:
            ang_univector = assert_angle(ang_univector + np.pi)
            response_lin *= -1

        self.controller_ang.set_point = ang_univector

        response_lin *= np.cos(assert_angle(ang_univector - actual_angle))**2

        response_ang = self.controller_ang.update(actual_angle)
        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), False)

