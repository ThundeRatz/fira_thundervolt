import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

class DefendCorner(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, y_position=0.45,
                    limit_dir = -0.5, ball_max_dist = 0.5):
        """
        Create an action node to follow the ball's x position on a horizontal line.

        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots.
            y_position (float): The horizontal position the will be maintined by the robot at every time. Defaults to 0.5.
            ball_max_dist (float, optional): Max dist the ball can stay away from the robot. Defauts to 0.5.
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.y_position = np.clip(y_position, data.GOAL_AREA_WIDTH/2, data.FIELD_WIDTH/2)
        self.limit_dir = limit_dir
        self.limit_esq = -data.FIELD_LENGTH/2
        self.ball_max_dist = ball_max_dist

        self.parameters.register_key('defend_upper', access=py_trees.common.Access.READ)

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.001, kd_ang=1.0, tolerance_ang=0.03,
                        kp_lin=200.0, ki_lin=0.001, kd_lin=2.0, tolerance_lin=0.005,
                        saturation_ang=(6*np.pi/6), max_integral_ang=np.pi/20, integral_fade_ang=0.75,
                        saturation_lin=(200 * (self.limit_dir - self.limit_esq)/2), max_integral_lin=1.0, integral_fade_lin=0.75,
                        line_dist_std_dev=0.03, linear_decay_std_dev=np.pi/30)

    def initialise(self):
        if self.parameters.defend_upper:
            line_y = self.y_position
        else:
            line_y = -self.y_position
        self.action.initialize(self.parameters.robot_id, (self.limit_esq, line_y), (self.limit_dir, line_y))

    def update(self):
        robot_position = np.array((self.field_data.robots[self.parameters.robot_id].position.x,
                            self.field_data.robots[self.parameters.robot_id].position.y))
        ball_position = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        self.action.set_goal(ball_position)
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if np.linalg.norm(robot_position - ball_position) > self.ball_max_dist:
            return py_trees.common.Status.FAILURE

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
