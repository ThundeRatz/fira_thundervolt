import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.data import FieldData, Pose2D
from thundervolt.core.command import RobotCommand
from thundervolt.core.utils import versor

class LineAction(Action):
    def __init__(self, kp_lin, ki_lin, kd_lin, tolerance, kp_ang, ki_ang, kd_ang):
        super().__init__()
        self.tolerance = tolerance
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin)
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang)


    def initialize(self, robot_id, pointA, pointB):
        super().initialize(robot_id)
        self.controller_lin.reset()
        self.controller_ang.reset()

        if pointB[0] < pointA[0]:
            pointA, pointB = pointB, pointA

        self.pointA = pointA
        self.pointB = pointB
        self.aux = (pointB[1] - pointA[1]) / (pointB[0] - pointA[0])

        self.controller_lin.set_point(0)

    def set_goal(self, pointC):
        u = pointC - self.pointA
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

    def update(self, field_data: FieldData) -> RobotCommand:
        actual_point = np.zeros(2)

        actual_point[0] = field_data.robots[self.robot_id].position.x
        actual_point[1] = field_data.robots[self.robot_id].position.y
        actual_ang = field_data.robots[self.robot_id].position.theta

        if np.linalg.norm(self.goal-actual_point) < self.tolerance:
            return (RobotCommand(), True)

        goal_ang = np.arctan2(actual_point-self.goal)
        self.controller_ang.set_point(goal_ang)

        response_ang = self.controller_ang.update(actual_ang)
        response_lin = self.controller_lin.update(np.linalg.norm(self.goal-actual_point))

        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), False)
