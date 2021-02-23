import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoNearCorner(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, side, goal_x, goal_y):

        """
        Create an action node to make the robot go near the corner kick area
        Args:
            name (string): name of the node
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            side (int, optional): decides if the point is in the upper (side = 1) or lower (side = 0) corner area
            goal_x (float, optional): Defines a x goal. Defaults to -0.65
            goal_y (float, optional): Defines a y goal. Defaults to -0.5
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.side = side
        self.goal_x = goal_x
        self.goal_y = goal_y

    def setup(self):
        self.action = FollowFieldAction(kp_ang=10.0, ki_ang=0.0, kd_ang=3.0, kp_lin=50.0, ki_lin=0.0,
                                        kd_lin=3.0, tolerance_lin=0.05, base_speed=20,
                                        goal=None, use_front=True)

    def initialise(self):
        if self.goal_x is None:
            self.goal_x = -0.65 # Default value
        if self.goal_y is None:
            self.goal_y = -0.5 # Default value

        if self.side == 1:
            self.goal_y *= -1

        # Field declarations
        self.attract_field = fields.RadialField(
            target = (self.goal_x, self.goal_y),
            max_radius = 3.0,
            decay_radius = 0.3,
            repelling = False,
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.3,
            decay_radius = 0.05,
            multiplier = 1,
        )

        area_field = combinations.AreaField(
            max_dist = 0.3,
            decay_dist = 1.0,
            multiplier = 3.5,
        )

        self.my_field = fields.VectorField()
        self.my_field.add(repell_field)
        self.my_field.add(self.attract_field)
        self.my_field.add(area_field)

        self.action.initialize(self.parameters.robot_id, self.my_field)

    def update(self):
        self.my_field.update(self.field_data, self.parameters.robot_id)
        self.action.set_goal(np.array([self.goal_x, self.goal_y]))

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
