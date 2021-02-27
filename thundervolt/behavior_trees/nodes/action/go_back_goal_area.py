import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields.fields import VectorField, RadialField, LineField, OrientedAttractingField
from thundervolt.vector_fields.combinations import WallField, ObstaclesField, TangentObstaclesField

from thundervolt.vector_fields.plotter import FieldPlotter

X_DIRECTION_TOLERANCE = 0.05
Y_DIRECTION_TOLERANCE = 0.15

class BackToGoalArea(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, x_position=-0.55):

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
        self.x_position = x_position

    def setup(self):
        self.vector_field = VectorField(name="Back to Goal!")

        avoid_area = LineField(
            target = (-data.FIELD_LENGTH / 2, 0),
            theta = -np.pi / 2,
            size = data.GOAL_AREA_WIDTH / 2 + 0.1,
            only_forward = False,
            side = 'negative',
            repelling = True,
            max_dist = 0.25,
            decay_dist = 0.01,
            multiplier = 0.5,
        )

        repelling_field = ObstaclesField(
            max_radius = 0.17,
            decay_radius = 0.1,
            multiplier = 1.0
        )

        avoid_obstacles = TangentObstaclesField(
                            radius = 1.5,
                            max_radius = 0.25,
                            decay_radius = 0.1,
                            multiplier = 1.0
        )

        self.attracting_field = OrientedAttractingField(
            target = (self.x_position, 0),
            direction=(0,1),
            nodes_radius = 0.1,
            damping = 1/250,
            max_radius = 2.0,
            decay_radius = 0.01,
            multiplier = 1.0
        )

        self.ball_repelling_field = RadialField(
            target = (self.field_data.ball.position.x, self.field_data.ball.position.y),
            max_radius = 0.25,
            decay_radius = 0.03,
            repelling = True,
            multiplier = 0.9
        )

        avoid_walls = WallField(
                        max_dist=0.1,
                        decay_dist=0.05,
                        multiplier=0.7
        )

        self.vector_field.add(self.attracting_field)
        self.vector_field.add(repelling_field)
        self.vector_field.add(avoid_obstacles)
        self.vector_field.add(avoid_area)
        self.vector_field.add(avoid_walls)
        self.vector_field.add(self.ball_repelling_field)

        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        base_speed=30, linear_decay_std_dev=np.pi/4)


    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        robot_pos = np.array((self.field_data.robots[self.parameters.robot_id].position.x, self.field_data.robots[self.parameters.robot_id].position.y))

        if abs(robot_pos[0] - self.x_position) < X_DIRECTION_TOLERANCE and abs(robot_pos[1] - ball_pos[1]) < Y_DIRECTION_TOLERANCE:
            return py_trees.common.Status.SUCCESS

        self.ball_repelling_field.target = ball_pos

        goal_position = (self.x_position, ball_pos[1])
        self.attracting_field.target = goal_position
        # self.action.set_goal(goal_position)
        if self.field_data.robots[self.parameters.robot_id].position.y > ball_pos[1]:
            self.attracting_field.direction = (0,-1)
        else:
            self.attracting_field.direction = (0,1)

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
