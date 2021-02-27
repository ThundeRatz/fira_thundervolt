import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.core import utils
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields.fields import VectorField, OrientedAttractingField
from thundervolt.vector_fields.combinations import ObstaclesField, TangentObstaclesField, WallField, AreaField

from thundervolt.vector_fields.plotter import FieldPlotter


class GetBallDefender(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, max_position = 0.15, min_position = -0.5):
        """
        Create an action node to try to get the ball
        Terminates when ball reaches attack field

        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots.
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command

        self.max_position = max_position
        self.min_position = min_position

    def setup(self):
        self.vector_field = VectorField(name='Get Ball Field')

        avoid_obstacles = TangentObstaclesField(
                            radius = 1.5,
                            max_radius = 0.25,
                            decay_radius = 0.1,
                            multiplier = 1.0)

        avoid_obstacles_2 = ObstaclesField(
                            max_radius=0.15,
                            decay_radius=0.05,
                            multiplier=0.9)

        avoid_walls = WallField(
                        max_dist=0.2,
                        decay_dist=0.05,
                        multiplier=0.9)

        area_field = AreaField(
            max_dist = 0.3,
            decay_dist = 1.0,
            multiplier = 1.0,
        )

        nodes_radius = 0.08
        base_speed = 45
        disable_threshold = 0.4
        enable_threshold = 0.35

        def update_get_ball_field(field, field_data, robot_id):
            ball_pos = np.array((field_data.ball.position.x, field_data.ball.position.y))
            ball_vel = np.array((field_data.ball.velocity.x, field_data.ball.velocity.y))

            robot_pos = np.array((field_data.robots[robot_id].position.x, field_data.robots[robot_id].position.y))
            approx_robot_speed = base_speed * data.WHEEL_RADIUS * 0.8
            ball_next_pos = ball_pos + ball_vel * (np.linalg.norm(robot_pos - ball_pos)) / approx_robot_speed

            ball_next_pos[0] = np.clip(ball_next_pos[0], -data.FIELD_LENGTH/2, data.FIELD_LENGTH/2)
            ball_next_pos[1] = np.clip(ball_next_pos[1], -data.FIELD_WIDTH/2, data.FIELD_WIDTH/2)

            ball_to_goal = np.array((data.FIELD_LENGTH/2, 0)) - ball_next_pos

            # Check if ball to goal direction leaves enough space for robot positioning
            allowed_direction = utils.versor(ball_to_goal) * (data.BALL_RADIUS + 2 * data.ROBOT_SIZE)
            dist_to_wall = data.FIELD_WIDTH/2 - abs(ball_next_pos[1])


            if allowed_direction[1] > 0:
                allowed_direction[1] = min(allowed_direction[1], dist_to_wall)
            else:
                allowed_direction[1] = max(allowed_direction[1], -dist_to_wall)


            if ball_pos[1] > disable_threshold:
                field.disable_positive = True
            if ball_pos[1] < enable_threshold:
                field.disable_positive = False
            if ball_pos[1] < -disable_threshold:
                field.disable_negative = True
            if ball_pos[1] > -enable_threshold:
                field.disable_negative = False

            field.target = ball_next_pos
            field.direction = allowed_direction

        get_ball = OrientedAttractingField(
                    target = (0, 0),
                    direction = (1, 0),
                    nodes_radius = nodes_radius,
                    damping = 1/250,
                    max_radius = 2.0,
                    decay_radius = 0.01,
                    multiplier = 1.5,
                    update_rule = update_get_ball_field)

        self.vector_field.add(avoid_obstacles)
        self.vector_field.add(avoid_obstacles_2)
        self.vector_field.add(avoid_walls)
        self.vector_field.add(area_field)
        self.vector_field.add(get_ball)

        self.action = FollowFieldAction(
                        kp_ang=7.0, ki_ang=0.005, kd_ang=2.0,
                        # kp_lin=200.0, ki_lin=0.03, kd_lin=3.0, tolerance_lin=0.005,
                        saturation_ang=(8*np.pi/3), integral_fade_ang=0.75,
                        # saturation_lin=(200*0.2), integral_fade_lin=0.75,
                        base_speed=base_speed, linear_decay_std_dev=np.pi/4)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.vector_field)

    def update(self):
        self.vector_field.update(self.field_data, self.parameters.robot_id)
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if self.field_data.ball.position.x > self.max_position:
            return py_trees.common.Status.SUCCESS

        elif self.field_data.ball.position.x < self.min_position:
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

    def plot_field(self):
        self.vector_field.update(self.field_data, self.parameters.robot_id)
        my_plotter = FieldPlotter('Radial Field')
        my_plotter.plot(self.vector_field)
