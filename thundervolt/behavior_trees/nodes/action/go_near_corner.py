import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoNearCorner(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, y_position=0.0,
                    min_sup_limit=data.FIELD_WIDTH/2 , max_inf_limit=-data.GOAL_AREA_WIDTH/2):

        """
        Create an action node to make the robot go near the corner kick area
        Args:
            name (string): node name
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            y_position (float, optional): y goal. Defaults to -0.45
            max_inf_limit (float, optional): max y value in bottom corner area. Defaults to -0.35
            min_sup_limit (float, optional): min y value in top corner area. Defaults to 0.35
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.y_position = y_position
        self.min_sup_limit = min_sup_limit
        self.max_inf_limit = max_inf_limit

    def setup(self):
        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        kp_lin=50.0, ki_lin=0.01, kd_lin=3.0, tolerance_lin=0.05,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        # saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4)

    def initialise(self):
        if self.y_position > 0 and self.y_position <= self.min_sup_limit:
            self.goal_y = self.min_sup_limit + 0.05

        elif self.y_position < 0 and self.y_position >= self.max_inf_limit:
            self.goal_y = self.max_inf_limit + 0.05

        elif self.y_position is None:
            self.goal_y = -0.45 # Default value

        else:
            self.goal_y = self.y_position

        self.goal_x = -0.6  # Initial value

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
