import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields import fields, combinations, plotter

class WaitToStrike(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):
        super().__init__(name, role, field_data)
        self.team_command = team_command


    def setup(self):
        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        kp_lin=200.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.005,
                        saturation_ang=(15*np.pi), integral_fade_ang=0.75,
                        saturation_lin=(200), integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4)


    def initialise(self):
        self.division_field = fields.LineField(
            target = (self.field_data.ball.position.x, 0),
            theta = np.pi / 2,
            size = data.FIELD_WIDTH / 2,
            side = 'positive',
            repelling = True,
            max_dist = data.ROBOT_SIZE,
            multiplier = 1.5
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.3,
            decay_radius = 0.05,
            multiplier = 0.8
        )

        self.vector_field = fields.VectorField()
        self.vector_field.add(self.division_field)
        self.vector_field.add(repell_field)
        self.action.initialize(self.parameters.robot_id, self.vector_field)


    def update(self):
        player_y = self.field_data.robots[self.parameters.robot_id].position.y
        ball_y = self.field_data.ball.position.y

        if ball_y - data.ROBOT_SIZE < player_y < ball_y + data.ROBOT_SIZE:
            if (data.FIELD_WIDTH/2 + ball_y - data.ROBOT_SIZE) <= (data.FIELD_WIDTH/2 - ball_y + data.ROBOT_SIZE):
                goal_y = ball_y + data.ROBOT_SIZE
            else:
                goal_y = ball_y - data.ROBOT_SIZE
        else:
            goal_y = player_y

        goal = (self.field_data.ball.position.x + data.ROBOT_SIZE, goal_y)

        self.vector_field.update(self.field_data, self.parameters.robot_id)
        self.division_field.target = (self.field_data.ball.position.x, 0)
        self.action.set_goal(np.array(goal))

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        pass


    def plot_field(self):
        self.vector_field.update(self.field_data, self.parameters.robot_id)
        my_plotter = plotter.FieldPlotter('Wait to Strike')
        my_plotter.plot(self.division_field)
        my_plotter.plot(self.vector_field)
