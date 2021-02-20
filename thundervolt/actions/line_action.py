import numpy as np

from .action import Action
from thundervolt.core.data import FieldData, Pose2D
from thundervolt.core.command import RobotCommand

class LineAction(Action):
    def __init__(self, kp, ki, kd, tolerance):
        super().__init__(kp, ki, kd, tolerance)

    def initialize(self, robot_id, pointA, pointB, pointBall):
        super().initialize(robot_id)
        goal = Pose2D()

        if pointB.x < pointA.x:
            pointA, pointB = pointB, pointA

        aux = (pointB.y - pointA.y) / (pointB.x - pointA.x)
        goal.x = (aux * (aux*pointA.x - pointA.y + pointBall.y) + pointBall.x) / (1 + aux**2)
        goal.y = (aux * (aux*pointB.y - pointA.x + pointBall.x) + pointB.x) / (1 + aux**2)

        if goal.x < pointA.x:
            goal = pointA
        elif goal.x > pointB.x:
            goal = pointB

        self.goal = goal

    def update(self, field_data: FieldData) -> RobotCommand:
        return
