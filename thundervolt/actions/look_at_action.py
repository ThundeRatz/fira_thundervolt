import numpy as np

from .action import Action
from thundervolt.core.utils import assert_angle
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand
from thundervolt.core.pid_controller import pidController

class LookAtAction(Action):
    def __init__(self, kp, ki, kd, tolerance,
                saturation=None, max_integral=None, integral_fade=None):
        """
        Create a look at action object

        Args:
            kp (float): Proportional constant for pid controller.
            ki (float): Integrative constant for pid controller.
            kd (float): Derivative constant for pid controller.
            tolerance (float): Settling interval around set point.
            saturation (float, optional): pid controller saturation.
            max_integral (float, optional): pid controller max integral value.
            integral_fade (float, optional): pid controller integral fade rate.
        """
        super().__init__()
        self.tolerance = tolerance
        self.controller = pidController(kp, ki, kd, saturation=saturation,
                            max_integral=max_integral, integral_fade_rate=integral_fade)


    def initialize(self, robot_id):
        super().initialize(robot_id)
        self.controller.reset()

    def set_angle(self, angle):
        self.final_angle = assert_angle(angle)

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        action_status = False
        last_received_angle = field_data.robots[self.robot_id].position.theta

        # Check if it is inside the tolerance
        if abs(assert_angle(self.final_angle - last_received_angle)) < self.tolerance:
            action_status = True

        # Calculate controller response
        response = self.controller.update(assert_angle(last_received_angle - self.final_angle))
        return (RobotCommand(-response, response), action_status)
