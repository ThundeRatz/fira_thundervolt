import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand
from thundervolt.core.utils import versor, assert_angle, rotate, gaussian

class LineAction(Action):
    def __init__(self, kp_ang, ki_ang, kd_ang, tolerance_ang, kp_lin, ki_lin, kd_lin, tolerance_lin,
                saturation_ang=None, max_integral_ang=None, integral_fade_ang=None,
                saturation_lin=None, max_integral_lin=None, integral_fade_lin=None,
                line_dist_std_dev=0.05, linear_decay_std_dev=None):
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
            line_dist_std_dev(float, optional): Distance to line standard deviation. Defaults to 0.05.
            linear_decay_std_dev (float, optional): Standard deviation for linear response gaussian decay function.
        """
        super().__init__()
        self.tolerance_ang = tolerance_ang
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang,
                                saturation=saturation_ang, max_integral=max_integral_ang, integral_fade_rate=integral_fade_ang)

        self.tolerance_lin = tolerance_lin
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin,
                                saturation=saturation_lin, max_integral=max_integral_lin, integral_fade_rate=integral_fade_lin)

        self.line_dist_std_dev = line_dist_std_dev
        self.linear_decay_std_dev = linear_decay_std_dev


    def initialize(self, robot_id, pointA, pointB):
        super().initialize(robot_id)
        self.controller_lin.reset()
        self.controller_ang.reset()

        if pointB[0] < pointA[0]:
            pointA, pointB = pointB, pointA

        # Save start and end point
        self.pointA = np.array(pointA)
        self.pointB = np.array(pointB)

        # Save the line basis
        self.line_dir = versor(self.pointB - self.pointA)
        self.axis_dir = rotate(self.line_dir, -np.pi/2)

    def set_goal(self, point_goal):
        # Calculates the goal projection on the line
        u = np.array(point_goal) - self.pointA
        u_proj_line = np.dot(u, self.line_dir)

        if u_proj_line < 0:
            goal = self.pointA
        elif u_proj_line > np.linalg.norm(self.pointB - self.pointA):
            goal = self.pointB
        else:
            goal = self.pointA + self.line_dir*u_proj_line

        self.goal = goal

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        action_status = False
        actual_point = np.zeros(2)

        # Get robot actual position
        actual_point[0] = field_data.robots[self.robot_id].position.x
        actual_point[1] = field_data.robots[self.robot_id].position.y
        actual_ang = field_data.robots[self.robot_id].position.theta

        # Calculate the robot to goal vector and coordinates ont the line basis
        goal_vector = self.goal - actual_point
        goal_proj_line = np.dot(goal_vector, self.line_dir)
        goal_proj_axis = np.dot(goal_vector, self.axis_dir)

        # Calculate the desired angle for the controller based on the robot to goal angle
        # and the line angle with weights based on the gaussian distribution of the robots distance
        # to the line
        if goal_proj_line < 0:
            line_ang = np.arctan2(-self.line_dir[1], -self.line_dir[0])
        else:
            line_ang = np.arctan2(self.line_dir[1], self.line_dir[0])

        goal_ang = np.arctan2(goal_vector[1], goal_vector[0])

        weight = gaussian(goal_proj_line, std_dev=self.line_dist_std_dev)
        desired_angle = line_ang * weight + goal_ang * (1-weight)

        # Calculate the linear response based on the robot to goal projection on the line direction
        response_lin = -self.controller_lin.update(abs(goal_proj_line))

        # Decide the robot side to use
        if abs(assert_angle(desired_angle - actual_ang)) > np.pi/2:
            desired_angle = assert_angle(desired_angle + np.pi)
            response_lin *= -1

        # Calculte the linear response decay based on the angular error gaussian distribution
        if self.linear_decay_std_dev is not None:
            response_lin *= gaussian(assert_angle(desired_angle - actual_ang), std_dev=self.linear_decay_std_dev)

        # Calculate the angular response
        angle_to_goal = assert_angle(actual_ang - desired_angle)
        response_ang = self.controller_ang.update(angle_to_goal)

        # Check tolerances
        if np.linalg.norm(abs(goal_proj_line)) < self.tolerance_lin and abs(assert_angle(desired_angle - actual_ang)) < self.tolerance_ang:
            action_status = True

        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), action_status)
