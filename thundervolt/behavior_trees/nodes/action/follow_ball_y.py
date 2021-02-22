import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

class FollowBallVertical(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, x_position=0.0,
                    limit_sup=data.FIELD_WIDTH/2 , limit_inf=-data.FIELD_WIDTH/2):

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.x_position = np.clip(x_position, -data.FIELD_LENGTH/2, data.FIELD_LENGTH/2)
        self.limit_sup = np.clip(limit_sup, -data.FIELD_WIDTH/2, data.FIELD_WIDTH/2)
        self.limit_inf = np.clip(limit_inf, -data.FIELD_WIDTH/2, data.FIELD_WIDTH/2)

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.005, kd_ang=2.0, tolerance_ang=0.03,
                        kp_lin=200.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.005,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        linear_decay_std_dev=np.pi/30)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, (self.x_position, self.limit_inf), (self.x_position, self.limit_sup))

    def update(self):
        ball_position = (self.field_data.ball.position.x, self.field_data.ball.position.y)
        self.action.set_goal(ball_position)
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
