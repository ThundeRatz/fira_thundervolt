import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.utils import assert_angle
from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand

class SpinAction(Action):
    def __init__(self, kp, ki, kd, tolerance,
                saturation=None, max_integral=None, integral_fade=None):
        """
        Create a spin action object

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

    def initialize(self, robot_id, turns):
        super().initialize(robot_id)
        self.controller.reset()
        self.total_angular_dist = turns * 2 * np.pi
        self.actual_angular_dist = 0
        self.last_angle = None
        self.controller.set_point = self.total_angular_dist

    def update(self, field_data: FieldData) -> (RobotCommand, bool):
        if self.last_angle is not None:
            self.actual_angular_dist += \
                assert_angle(field_data.robots[self.robot_id].position.theta - self.last_angle)

        self.last_angle = field_data.robots[self.robot_id].position.theta

        # Check if it has passsed the set point
        if abs(self.actual_angular_dist) > abs(self.total_angular_dist):
            return (RobotCommand(), True)

        # Check if it is inside the tolerance
        if abs(self.total_angular_dist - self.actual_angular_dist) < self.tolerance:
            return (RobotCommand(), True)

        # Calculate controller response
        response = self.controller.update(self.actual_angular_dist)
        return (RobotCommand(-response, response), False)
