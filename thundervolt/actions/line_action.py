import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.data import FieldData, Pose2D
from thundervolt.core.command import RobotCommand
from thundervolt.core.utils import versor, assert_angle

class LineAction(Action):
    def __init__(self, kp_lin, ki_lin, kd_lin, tolerance_lin, kp_ang, ki_ang, kd_ang, tolerance_ang):
        super().__init__()
        self.tolerance_lin = tolerance_lin
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin)
        self.tolerance_ang = tolerance_ang
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang)


    def initialize(self, robot_id, pointA, pointB):
        super().initialize(robot_id)
        self.controller_lin.reset()
        self.controller_ang.reset()

        if pointB[0] < pointA[0]:
            pointA, pointB = pointB, pointA

        self.pointA = pointA
        self.pointB = pointB

        self.controller_lin.set_point = 0

    def set_goal(self, point_goal):
        u = point_goal - self.pointA
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
        actual_point = np.zeros(2)

        actual_point[0] = field_data.robots[self.robot_id].position.x
        actual_point[1] = field_data.robots[self.robot_id].position.y
        actual_ang = field_data.robots[self.robot_id].position.theta

        goal_vector = self.goal - actual_point

        line = self.pointA - self.pointB

        if np.linalg.norm(goal_vector) < self.tolerance_lin:
            goal_ang = np.arctan2(line[1], line[0])
            if abs(goal_ang - actual_ang) < self.tolerance_ang:
                return (RobotCommand(), True)
            response_lin = 0
        else:
            goal_ang = np.arctan2(goal_vector[1], goal_vector[0])
            response_lin = -self.controller_lin.update(np.linalg.norm(goal_vector))

        if abs(assert_angle(goal_ang - actual_ang)) > np.pi/2:
            goal_ang = assert_angle(goal_ang + np.pi)
            response_lin *= -1

        response_lin *= np.cos(assert_angle(goal_ang - actual_ang))**2

        distance_to_goal = actual_ang - goal_ang
        response_ang = self.controller_ang.update(distance_to_goal)

        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), False)
