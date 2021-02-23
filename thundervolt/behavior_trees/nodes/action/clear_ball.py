import py_trees

from ..execution_node import ExecutionNode
from thundervolt.actions.spin_action import SpinAction
from thundervolt.core.utils import assert_half_angle
from thundervolt.core.data import ROBOT_SIZE


class ClearBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, num_of_turns):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.num_of_turns = num_of_turns

    def setup(self):
        self.action = SpinAction(kp=9.0, ki=0.02, kd=1.0, tolerance=0.05)

    def initialise(self):
        robot_ang = self.field_data.robots[self.parameters.robot_id].position.theta
        robot_y_position = self.field_data.robots[self.parameters.robot_id].position.y
        ball_y_position = self.field_data.ball.position.y

        rotation_direction = 1 if (robot_y_position > ball_y_position) else -1

        self.action.initialize(self.parameters.robot_id, rotation_direction * self.num_of_turns)

    def update(self):
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

