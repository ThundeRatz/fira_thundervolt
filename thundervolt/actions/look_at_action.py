import numpy as np

from .action import Action
from thundervolt.core.utils import assert_angle
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand
from thundervolt.core.pid_controller import pidController

class LookAtAction(Action):
    def __init__(self, kp, ki, kd, tolerance):
        super().__init__()
        self.tolerance = tolerance
        self.controller = pidController(kp, ki, kd)

    def initialize(self, robot_id, angle):
        super().initialize(robot_id)
        self.final_angle = assert_angle(angle)
        self.last_received_angle = None
        self.controller.reset()
        self.controller.set_point = self.final_angle

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        self.last_received_angle = field_data.robots[self.robot_id].position.theta

        # Check if it is inside the tolerance
        if abs(self.final_angle - self.last_received_angle) < self.tolerance:
            return (RobotCommand(), True)

        # Calculate controller response
        response = self.controller.update(self.last_received_angle)
        return (RobotCommand(-response, response), False)
