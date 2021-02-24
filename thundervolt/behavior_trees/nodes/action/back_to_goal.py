import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields.fields import VectorField, RadialField
from thundervolt.vector_fields.combinations import ObstaclesField

from thundervolt.vector_fields.plotter import FieldPlotter

GOAL_LINE_X = data.FIELD_LENGTH/2 - data.ROBOT_SIZE*0.75
GOAL_LINE_Y = data.GOAL_WIDTH/2 - data.ROBOT_SIZE/4

class BackToGoal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):

        """
        Action node to go back to goal
        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots (rad/s).
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.vector_field = VectorField(name="Back to Goal!")

        repelling_field = ObstaclesField(
            max_radius = 0.25,
            decay_radius = 0.05,
            multiplier = 0.8,
        )

        self.attracting_field = RadialField(
            target = (-GOAL_LINE_X, 0),
            max_radius = 2.0,
            decay_radius = 0.3,
            repelling = False,
            multiplier=1.0
        )

        self.ball_repelling_field = RadialField(
            target = (self.field_data.ball.position.x, self.field_data.ball.position.y),
            max_radius = 0.25,
            decay_radius = 0.05,
            repelling = True,
            multiplier = 0.8
        )

        self.vector_field.add(self.attracting_field)
        self.vector_field.add(repelling_field)
        self.vector_field.add(self.ball_repelling_field)

        self.action = FollowFieldAction(
                        kp_ang=9.0, ki_ang=0.009, kd_ang=2.0,
                        kp_lin=200.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.10,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        saturation_lin=(200*0.2), max_integral_lin=1, integral_fade_lin=0.75,
                        base_speed=300, linear_decay_std_dev=np.pi/6, goal=(-GOAL_LINE_X, 0)
        )

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
        self.ball_repelling_field.target = (self.field_data.ball.position.x, self.field_data.ball.position.y)

        goal_y_position = np.clip(self.field_data.ball.position.y, -GOAL_LINE_Y, GOAL_LINE_Y)
        goal_position = (-GOAL_LINE_X, goal_y_position)
        self.attracting_field.target = goal_position
        self.action.set_goal(goal_position)

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
        my_plotter = FieldPlotter('Back to goal field!')
        my_plotter.plot(self.vector_field)
