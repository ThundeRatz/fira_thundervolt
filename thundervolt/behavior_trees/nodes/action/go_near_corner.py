import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoNearCorner(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, upper):

        """
        Create an action node to make the robot go near the corner kick area
        Args:
            name (string): node name
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            upper (bool, optional): decides if the point is in the upper or lower corner area when the goal
                                    isn´t defined. Defaults to false
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.upper = upper

    def setup(self):
        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        kp_lin=50.0, ki_lin=0.01, kd_lin=3.0, tolerance_lin=0.05,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        # saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4)

    def initialise(self):
        self.goal_y = -0.45 # Default value
        self.goal_x = -0.6  # Initial value

        if self.upper is True:
            self.goal_y = abs(self.goal_y)

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
            multiplier = 1,
        )

        area_field = combinations.AreaField(
            max_dist = 0.3,
            decay_dist = 1.0,
            multiplier = 3.5,
        )

        self.vector_field = fields.VectorField()
        self.vector_field.add(repell_field)
        self.vector_field.add(self.attract_field)
        self.vector_field.add(area_field)

        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
        self.vector_field.update(self.field_data, self.parameters.robot_id)
        if self.field_data.ball.position.x < -data.FIELD_LENGTH/3:
            self.goal_x = self.field_data.ball.position.x
        self.action.set_goal(np.array([self.goal_x, self.goal_y]))
        self.attract_field.target = (self.goal_x, self.goal_y)

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
