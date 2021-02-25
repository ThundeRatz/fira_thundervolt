import numpy as np
from numpy.core.defchararray import multiply
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields import fields, combinations, plotter

class WaitToStrike(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, x_partition):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.x_partition = x_partition


    def setup(self):
        line_field_dist = abs(-data.FIELD_LENGTH / 2 - self.x_partition)

        division_field = fields.LineField(
            target = (-data.FIELD_LENGTH / 2, 0),
            theta = np.pi / 2,
            size = data.FIELD_WIDTH / 2,
            side = 'positive',
            repelling = True,
            max_dist = line_field_dist,
            decay_dist = line_field_dist * 0.8,
            multiplier = 0.7
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.17,
            decay_radius = 0.08,
            multiplier = 0.9
        )

        avoid_obstacles = combinations.TangentObstaclesField(
                            radius = 0.7,
                            max_radius = 0.3,
                            decay_radius = 0.08,
                            multiplier = 1.2
        )

        wall_field = combinations.WallField(
                        max_dist=0.1,
                        decay_dist=0.05,
                        multiplier=0.7
        )

        self.attracting_field = fields.RadialField(
            target = (self.x_partition, self.field_data.ball.position.y + 2 * data.ROBOT_SIZE),
            max_radius = 2.0,
            decay_radius = 0.3,
            repelling = False,
            multiplier = 0.9
        )

        self.vector_field = fields.VectorField()
        self.vector_field.add(division_field)
        self.vector_field.add(avoid_obstacles)
        self.vector_field.add(repell_field)
        self.vector_field.add(wall_field)
        self.vector_field.add(self.attracting_field)

        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.01, kd_ang=2.0,
                        kp_lin=150.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.1,
                        saturation_ang=(100*np.pi), integral_fade_ang=0.75,
                        saturation_lin=(100), max_integral_lin=0.5, integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4, use_front=True)


    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.vector_field)
        self.goal_y = None


    def update(self):
        ball_y = self.field_data.ball.position.y

        if self.goal_y is None:
            if ball_y <= 0:
                self.goal_y = ball_y + 3 * data.ROBOT_SIZE
            else:
                self.goal_y = ball_y - 3 * data.ROBOT_SIZE
        elif self.goal_y > 0:
            if ball_y <= -data.ROBOT_SIZE:
                self.goal_y = ball_y + 3 * data.ROBOT_SIZE
            else:
                self.goal_y = ball_y - 3 * data.ROBOT_SIZE
        elif self.goal_y <= 0:
            if ball_y > data.ROBOT_SIZE:
                self.goal_y = ball_y - 3 * data.ROBOT_SIZE
            else:
                self.goal_y = ball_y + 3 * data.ROBOT_SIZE

        goal = (self.x_partition + data.ROBOT_SIZE, self.goal_y)
        self.attracting_field.target = goal
        self.action.set_goal(np.array(goal))
        self.vector_field.update(self.field_data, self.parameters.robot_id)

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
        # my_plotter.plot(self.attracting_field)
        my_plotter.plot(self.vector_field)
