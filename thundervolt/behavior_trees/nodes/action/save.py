import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

LIMIT_VELOCITY = 0.5
MIN_DIST = data.ROBOT_SIZE

class SaveGoal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, save_time, goal_line_x, goal_line_y):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.save_time = save_time
        self.goal_line_x = goal_line_x
        self.goal_line_y = goal_line_y

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.001, kd_ang=3.0, tolerance_ang=0.03,
                        kp_lin=350.0, ki_lin=0.001, kd_lin=2.0, tolerance_lin=0.01,
                        saturation_ang=(6*np.pi/6), max_integral_ang=np.pi/20, integral_fade_ang=0.75,
                        saturation_lin=None, max_integral_lin=1.0, integral_fade_lin=None,
                        line_dist_std_dev=0.03, linear_decay_std_dev=np.pi/30)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, (-self.goal_line_x, -self.goal_line_y), (-self.goal_line_x, self.goal_line_y))

    def update(self):
        ball_position = (self.field_data.ball.position.x, self.field_data.ball.position.y)
        ball_velocity = (self.field_data.ball.velocity.x, self.field_data.ball.velocity.y)
        ball_distance = -(ball_position[0] + data.FIELD_LENGTH)

        if ball_velocity[0] < 0:
            ball_time = ball_distance / ball_velocity[0]
            if ball_time < self.save_time:
                ball_distance = min(-(ball_position[0] + self.goal_line_x - data.ROBOT_SIZE/2), 0)
                ball_time = ball_distance / ball_velocity[0]
                goal_point = np.zeros(2)
                goal_point[1] = ball_position[1] + ball_time * ball_velocity[1]
                goal_point[1] = np.clip(goal_point[1], -data.GOAL_WIDTH/2, data.GOAL_WIDTH/2)

                player_to_ball = goal_point[1] - self.field_data.robots[self.parameters.robot_id].position.y

                if ((ball_time == 0 and abs(self.field_data.ball.position.y) < data.GOAL_AREA_WIDTH/2) or abs(player_to_ball/ball_time) > LIMIT_VELOCITY):
                    goal_point[1] += player_to_ball
                    self.action.controller_lin.kp = 1000.0
                    print("Dificil", player_to_ball)
                else:
                    self.action.controller_lin.kp = 350.0

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
