import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

class SaveGoal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):
        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.005, kd_ang=2.0, tolerance_ang=0.03,
                        kp_lin=200.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.005,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        linear_decay_std_dev=np.pi/30)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, (-0.70, -0.18), (-0.70, 0.18))

    def update(self):
        ball_position = (self.field_data.ball.position.x, self.field_data.ball.position.y)
        ball_velocity = (self.field_data.ball.velocity.x, self.field_data.ball.velocity.y)
        ball_distance = -(ball_position[0] + data.FIELD_LENGTH/2)

        if ball_velocity[0] < 0:
            ball_time = ball_distance / ball_velocity[0]
            if ball_time < 0.5:
                goal_point = np.clip(ball_time * ball_velocity[1], -data.FIELD_WIDTH/2, data.FIELD_WIDTH/2)
                self.action.set_goal(goal_point)

                robot_cmd, action_status = self.action.update(self.field_data)
                self.team_command.commands[self.parameters.robot_id] = robot_cmd

                if action_status:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass
