import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields.fields import VectorField, OrientedAttractingField
from thundervolt.vector_fields.combinations import WallField, ObstaclesField, TangentObstaclesField

from thundervolt.vector_fields.plotter import FieldPlotter

class GoToFoesGoal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):

        """
        Action node to go to foes goal
        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots (rad/s).
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.vector_field = VectorField(name="Go to Foes Goal!")

        repelling_field = ObstaclesField(
            max_radius = 0.17,
            decay_radius = 0.08,
            multiplier = 0.9
        )

        avoid_obstacles = TangentObstaclesField(
                            radius = 0.7,
                            max_radius = 0.3,
                            decay_radius = 0.08,
                            multiplier = 1.2
        )

        attracting_field = OrientedAttractingField(
            target = (data.FIELD_LENGTH/2 + data.GOAL_DEPTH, 0.0),
            direction=(1.0, 0.0),
            nodes_radius=0.1,
        )

        avoid_walls = WallField(
                        max_dist=0.1,
                        decay_dist=0.05,
                        multiplier=0.7
        )

        self.vector_field.add(attracting_field)
        self.vector_field.add(repelling_field)
        self.vector_field.add(avoid_obstacles)
        self.vector_field.add(avoid_walls)
        #self.vector_field.add(goal_line_field)

        self.action = FollowFieldAction(
                        kp_ang=9.0, ki_ang=0.009, kd_ang=2.5,
                        kp_lin=200.0, ki_lin=0.01, kd_lin=3.0, tolerance_lin=data.GOAL_DEPTH+data.ROBOT_SIZE/2+data.BALL_RADIUS,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        saturation_lin=(200*0.2), max_integral_lin=0.5, integral_fade_lin=0.75,
                        base_speed=300, linear_decay_std_dev=np.pi/6, use_front=False, goal=(data.FIELD_LENGTH/2+data.GOAL_DEPTH, 0)
        )

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
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
