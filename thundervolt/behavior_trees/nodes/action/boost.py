import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields.fields import RadialField

class BoostBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):
        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.action = FollowFieldAction(kp_ang=10.0, ki_ang=0.005, kd_ang=3.0, kp_lin=100.0, ki_lin=0.005, kd_lin=3.0, tolerance_lin=0.07,
                                base_speed=20, goal=None, use_front=True, saturation_ang=None, saturation_lin=10.0)

    def initialise(self):
        self.vector_field = RadialField(target=(0, 0))
        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
        ball_point = np.array([self.field_data.ball.position.x, self.field_data.ball.position.y])
        self.vector_field.target = ball_point
        self.action.set_goal(ball_point)

        robot_cmd, action_state = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_state:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
