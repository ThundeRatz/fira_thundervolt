import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoBehindBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, distance, left_x_lim = -data.FIELD_LENGTH/2 + 2*data.GOAL_DEPTH):

        """
        Create an action node to make the robot go behind ball's x position
        Args:
            name (string): name of the node
            role (string): role of the player (/goalkeeper, /defender or /attacker)
            field_data (FieldData): information received from the field (e.g.: position of each player and ball)
            team_command (TeamCommand): velocity commands for a robot
            distance (float): desired distance between player and ball
            left_x_lim (floar, optional): left x limit. Default to -0.55
        """
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.distance = distance
        self.left_x_lim = left_x_lim

    def setup(self):
        self.action = FollowFieldAction(kp_ang=6.0, ki_ang=0.0005, kd_ang=2.0,
                        kp_lin=50.0, ki_lin=0.0, kd_lin=3.0, tolerance_lin=0.1,
                        saturation_ang=(8*np.pi/3), max_integral_ang=0.01, integral_fade_ang=0.5,
                        saturation_lin=(50*0.2), max_integral_lin=0.5, integral_fade_lin=0.5,
                        use_front = False, linear_decay_std_dev=np.pi/4)


    def initialise(self):
        goal_x = self.field_data.ball.position.x - self.distance
        # Field declarations
        self.attract_field = fields.RadialField(
            target = (goal_x, self.field_data.ball.position.y),
            max_radius = 3.0,
            decay_radius = 0.3,
            repelling = False,
        )

        avoid_obstacles = combinations.TangentObstaclesField(
                            radius = 1.5,
                            max_radius = 0.25,
                            decay_radius = 0.1,
                            multiplier = 1.0)

        avoid_colision = combinations.ObstaclesField(
                            max_radius=0.15,
                            decay_radius=0.05,
                            multiplier=0.9)

        avoid_walls = combinations.WallField(
                        max_dist=0.2,
                        decay_dist=0.05,
                        multiplier=0.9)

        self.ball_repell_field = fields.RadialField(
            target = (self.field_data.ball.position.x, self.field_data.ball.position.y),
            max_radius = 0.2,
            decay_radius = 0.05,
            repelling = True,
        )

        self.my_field = fields.VectorField()
        self.my_field.add(avoid_obstacles)
        self.my_field.add(avoid_colision)
        self.my_field.add(avoid_walls)
        self.my_field.add(self.ball_repell_field)
        self.my_field.add(self.attract_field)

        self.action.initialize(self.parameters.robot_id, self.my_field)

    def update(self):
        self.ball_repell_field.target = (self.field_data.ball.position.x, self.field_data.ball.position.y)

        goal_x = max(self.field_data.ball.position.x - self.distance, self.left_x_lim) # Near our goal area

        self.action.set_goal(np.array([goal_x, self.field_data.ball.position.y]))
        self.attract_field.target = (goal_x, self.field_data.ball.position.y)
        self.my_field.update(self.field_data, self.parameters.robot_id)

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
