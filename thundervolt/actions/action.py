from abc import ABC, abstractmethod

from thundervolt.core.data import FieldData
from thundervolt.core.command import RobotCommand

class Action(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def initialize(self, robot_id):
        self.robot_id = robot_id

    @abstractmethod
    def update(self, field_data : FieldData) -> (RobotCommand, bool):
        pass
