import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields.plotter import FieldPlotter
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoNearCorner(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, y_position = 0.45, limit_dir = -0.5):

        """
        Create an action node to make the robot go near the corner kick area
        Args:
            name (string): node name
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            y_position (float, optional): initial y position
        """

        super().__init__(name, role, field_data)
        self.goal_y = np.clip(abs(y_position), data.GOAL_AREA_WIDTH/2, data.FIELD_WIDTH/2)

        self.goal_x = -(data.FIELD_LENGTH/2 - data.GOAL_AREA_DEPTH)  # Initial value
        self.team_command = team_command

        self.limit_dir = limit_dir

        self.parameters.register_key('defend_upper', access=py_trees.common.Access.WRITE)

    def setup(self):

        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        kp_lin=50.0, ki_lin=0.01, kd_lin=3.0, tolerance_lin=0.1,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        # saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4)

        # Field declarations
        self.attract_field = fields.RadialField(
            target = (self.goal_x, self.goal_y),
            max_radius = 3.0,
            decay_radius = 0.3,
            repelling = False,
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.2,
            decay_radius = 0.1,
            multiplier = 0.8,
        )

        avoid_area = fields.LineField(
                        target = -data.FIELD_LENGTH/2 - data.GOAL_DEPTH,
                        theta = np.pi/2,
                        size = data.GOAL_AREA_WIDTH/2,
                        repelling = True,
                        max_dist = data.GOAL_DEPTH + data.GOAL_AREA_DEPTH + data.ROBOT_SIZE,
                        decay_dist = data.GOAL_DEPTH + data.GOAL_AREA_DEPTH,
                        multiplier = 1)

        self.vector_field = fields.VectorField()
        self.vector_field.add(repell_field)
        self.vector_field.add(self.attract_field)
        self.vector_field.add(avoid_area)


    def initialise(self):
        if self.field_data.ball.position.y > 0.0:
            upper = True
            self.goal_y = abs(self.goal_y)
        else:
            upper = False
            self.goal_y = -abs(self.goal_y)

        self.parameters.defend_upper = upper

        self.action.initialize(self.parameters.robot_id, self.vector_field)


    def update(self):


        self.goal_x = np.clip(self.field_data.ball.position.x, -data.FIELD_LENGTH/2, self.limit_dir)

        self.action.set_goal(np.array([self.goal_x, self.goal_y]))

        self.attract_field.target = (self.goal_x, self.goal_y)
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
        my_plotter = FieldPlotter('Go near corner Field')
        my_plotter.plot(self.vector_field)
