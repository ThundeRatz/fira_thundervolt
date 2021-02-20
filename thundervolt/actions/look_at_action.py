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
        self.controller.set_point = 0.0
        self.controller.saturation = kp * np.pi

    def initialize(self, robot_id):
        super().initialize(robot_id)
        self.controller.reset()

    def set_angle(self, angle):
        self.final_angle = assert_angle(angle)

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        last_received_angle = field_data.robots[self.robot_id].position.theta

        # Check if it is inside the tolerance
        if abs(assert_angle(self.final_angle - last_received_angle)) < self.tolerance:
            return (RobotCommand(), True)

        # Calculate controller response
        response = self.controller.update(assert_angle(last_received_angle - self.final_angle))
        return (RobotCommand(-response, response), False)
