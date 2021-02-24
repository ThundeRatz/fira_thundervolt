import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

GOAL_LINE_X = data.FIELD_LENGTH/2 - data.ROBOT_SIZE
GOAL_LINE_Y = data.GOAL_AREA_WIDTH/2 - data.ROBOT_SIZE

class SaveGoal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, save_time):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.save_time = save_time

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.001, kd_ang=3.0, tolerance_ang=0.03,
                        kp_lin=350.0, ki_lin=0.001, kd_lin=2.0, tolerance_lin=0.005,
                        saturation_ang=(6*np.pi/6), max_integral_ang=np.pi/20, integral_fade_ang=0.75,
                        saturation_lin=(350 * GOAL_LINE_Y), max_integral_lin=1.0, integral_fade_lin=0.75,
                        line_dist_std_dev=0.03, linear_decay_std_dev=np.pi/30)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, (-GOAL_LINE_X, -GOAL_LINE_Y), (-GOAL_LINE_X, GOAL_LINE_Y))

    def update(self):
        ball_position = (self.field_data.ball.position.x, self.field_data.ball.position.y)
        ball_velocity = (self.field_data.ball.velocity.x, self.field_data.ball.velocity.y)
        ball_distance = -(ball_position[0] + data.FIELD_LENGTH)

        if ball_velocity[0] < 0:
            ball_time = ball_distance / ball_velocity[0]
            if ball_time < self.save_time:
                ball_distance = min(-(ball_position[0] + GOAL_LINE_X - data.ROBOT_SIZE/2), 0)
                ball_time = ball_distance / ball_velocity[0]
                goal_point = np.zeros(2)
                goal_point[1] = ball_position[1] + ball_time * ball_velocity[1]
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
