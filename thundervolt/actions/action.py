from abc import ABC, abstractmethod

from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand
from thundervolt.core.pid_controller import pidController

class Action(ABC):
    def __init__(self, kp, ki, kd, tolerance):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tolerance = tolerance

    @abstractmethod
    def initialize(self, robot_id):
        self.robot_id = robot_id
        self.controller = pidController(self.kp, self.ki, self.kd)

    @abstractmethod
    def update(self, field_data : FieldData) -> (RobotCommand, bool):
        pass
