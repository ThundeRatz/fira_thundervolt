import numpy as np

from .action import Action
from thundervolt.core.math import assert_angle
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand

class SpinAction(Action):
    def __init__(self, kp, ki, kd, tolerance):
        super().__init__(kp, ki, kd, tolerance)

    def initialize(self, robot_id, turns):
        super().initialize(robot_id)
        self.total_angular_dist = turns * 2 * np.pi
        self.actual_angular_dist = 0
        self.last_angle = None
        print(type(self.controller))
        self.controller.set_point(self.total_angular_dist)

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        if self.last_angle:
            self.actual_angular_dist += \
                assert_angle(field_data.robots[self.robot_id].position.theta - self.last_angle)

        self.last_angle = field_data.robots[self.robot_id].position.theta

        if self.total_angular_dist < 0 and self.actual_angular_dist < self.total_angular_dist:
            return (RobotCommand(), True)

        if self.total_angular_dist > 0 and self.actual_angular_dist > self.total_angular_dist:
            return (RobotCommand(), True)

        if abs(self.total_angular_dist - self.actual_angular_dist) < self.tolerance:
            return (RobotCommand(), True)

        response = self.controller.update(self.actual_angular_dist)
        return (RobotCommand(-response, response), False)
