import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand
from thundervolt.core.utils import versor, assert_angle, gaussian

class LineAction(Action):
    def __init__(self, kp_ang, ki_ang, kd_ang, tolerance_ang, kp_lin, ki_lin, kd_lin, tolerance_lin,
                saturation_ang=None, max_integral_ang=None, integral_fade_ang=None,
                saturation_lin=None, max_integral_lin=None, integral_fade_lin=None,
                linear_decay_std_dev=None):
        """
        Create a line action object

        Args:
            kp_ang (float): Proportional constant for angular error
            ki_ang (float): Integrative constant for angular error
            kd_ang (float): Derivative constant for angular error
            tolerance_ang (float): Settling interval around set point.
            kp_lin (float): Proportional constant for linear error.
            ki_lin (float): Integrative constant for linear error.
            kd_lin (float): Derivative constant for linear error.
            tolerance_lin (float): Settling interval around set point.
            saturation_ang (float, optional): Angular pid controller saturation.
            max_integral_ang (float, optional): Angular pid controller max integral value.
            integral_fade_ang (float, optional): Angular pid controller integral fade rate.
            saturation_lin (float, optional): Linear pid controller saturation.
            max_integral_lin (float, optional): Linear pid controller max integral value.
            integral_fade_lin (float, optional): Linear pid controller integral fade rate.
            linear_decay_std_dev (float, optional): Standard deviation for linear response gaussian decay function.
        """
        super().__init__()
        self.tolerance_ang = tolerance_ang
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang,
                                saturation=saturation_ang, max_integral=max_integral_ang, integral_fade_rate=integral_fade_ang)

        self.tolerance_lin = tolerance_lin
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin,
                                saturation=saturation_lin, max_integral=max_integral_lin, integral_fade_rate=integral_fade_lin)

        self.linear_decay_std_dev = linear_decay_std_dev


    def initialize(self, robot_id, pointA, pointB):
        super().initialize(robot_id)
        self.controller_lin.reset()
        self.controller_ang.reset()

        if pointB[0] < pointA[0]:
            pointA, pointB = pointB, pointA

        self.pointA = np.array(pointA)
        self.pointB = np.array(pointB)

    def set_goal(self, point_goal):
        u = np.array(point_goal) - self.pointA
        v = self.pointB - self.pointA
        v_versor = versor(v)
        u_dot_v = np.dot(u, v_versor)

        if u_dot_v < 0:
            goal = self.pointA
        elif u_dot_v > np.linalg.norm(v):
            goal = self.pointB
        else:
            goal = self.pointA + v_versor*u_dot_v

        self.goal = goal

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        action_status = False
        actual_point = np.zeros(2)

        actual_point[0] = field_data.robots[self.robot_id].position.x
        actual_point[1] = field_data.robots[self.robot_id].position.y
        actual_ang = field_data.robots[self.robot_id].position.theta

        goal_vector = self.goal - actual_point

        line = self.pointA - self.pointB

        if np.linalg.norm(goal_vector) < self.tolerance_lin:
            goal_ang = np.arctan2(line[1], line[0])
            response_lin = 0
        else:
            goal_ang = np.arctan2(goal_vector[1], goal_vector[0])
            response_lin = -self.controller_lin.update(np.linalg.norm(goal_vector))

        if abs(assert_angle(goal_ang - actual_ang)) > np.pi/2:
            goal_ang = assert_angle(goal_ang + np.pi)
            response_lin *= -1

        if self.linear_decay_std_dev is not None:
            response_lin *= gaussian(assert_angle(goal_ang - actual_ang), std_dev=self.linear_decay_std_dev)

        angle_to_goal = assert_angle(actual_ang - goal_ang)
        response_ang = self.controller_ang.update(angle_to_goal)

        if response_lin == 0 and abs(assert_angle(goal_ang - actual_ang)) < self.tolerance_ang:
            action_status = True

        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), action_status)
