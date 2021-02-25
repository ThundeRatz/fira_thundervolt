import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoBehindBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, distance):

        """
        Create an action node to make the robot go behind ball's x position
        Args:
            name (string): name of the node
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            distance (float): desired distance between player and ball
        """
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.distance = distance

    def setup(self):
        self.action = FollowFieldAction(kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        kp_lin=50.0, ki_lin=0.01, kd_lin=3.0, tolerance_lin=0.05,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        # saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        base_speed=40, linear_decay_std_dev=np.pi/4)


    def initialise(self):
        goal_x = self.field_data.ball.position.x - self.distance
        # Field declarations
        self.attract_field = fields.RadialField(
            target = (goal_x, self.field_data.ball.position.y),
            max_radius = 3.0,
            decay_radius = 0.3,
            repelling = False,
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.15,
            decay_radius = 0.05,
            multiplier = 0.9,
        )

        self.ball_repell_field = fields.RadialField(
            target = (self.field_data.ball.position.x, self.field_data.ball.position.y),
            max_radius = 0.4,
            decay_radius = 0.05,
            repelling = True,
        )

        self.my_field = fields.VectorField()
        self.my_field.add(repell_field)
        self.my_field.add(self.ball_repell_field)
        self.my_field.add(self.attract_field)

        self.action.initialize(self.parameters.robot_id, self.my_field)

    def update(self):
        self.my_field.update(self.field_data, self.parameters.robot_id)
        self.ball_repell_field.target = (self.field_data.ball.position.x, self.field_data.ball.position.y)

        goal_x = max(self.field_data.ball.position.x - self.distance, -0.55) # Near our goal area

        self.action.set_goal(np.array([goal_x, self.field_data.ball.position.y]))
        self.attract_field.target = (goal_x, self.field_data.ball.position.y)

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
