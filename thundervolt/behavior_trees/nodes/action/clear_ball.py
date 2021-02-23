import py_trees
import numpy as np

from ..execution_node import ExecutionNode
from thundervolt.actions.spin_action import SpinAction
from thundervolt.core.data import ROBOT_SIZE


class ClearBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):
        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.action = SpinAction(kp=10.0, ki=0.01, kd=1.5, tolerance=0.05, saturation=2*np.pi*10)

    def initialise(self):
        robot_y_position = self.field_data.robots[self.parameters.robot_id].position.y
        ball_y_position = self.field_data.ball.position.y

        rotation_direction = 1 if (robot_y_position > ball_y_position) else -1

        self.action.initialize(self.parameters.robot_id, rotation_direction * 2.0)

    def update(self):
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

