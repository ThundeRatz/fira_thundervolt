import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.line_action import LineAction

class FollowBallHorizontal(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, y_position=0.0,
                    limit_dir=data.FIELD_LENGTH/2 , limit_esq=-data.FIELD_LENGTH/2):
        """
        Create an action node to follow the ball's x position on a horizontal line.

        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots.
            y_position (float): The horizontal position the will be maintined by the robot at every time.
            limit_dir (float, optional): Right limit value the robot can reach. Defualts to FIELD_LENGTH/2.
            limit_esq (float, optional): Min vertical value the robot can reach. Defualts to -FIELD_LENGTH/2.
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.y_position = np.clip(y_position, -data.FIELD_WIDTH/2, data.FIELD_WIDTH/2)
        self.limit_dir = np.clip(limit_dir, -data.FIELD_LENGTH/2, data.FIELD_LENGTH/2)
        self.limit_esq = np.clip(limit_esq, -data.FIELD_LENGTH/2, data.FIELD_LENGTH/2)

    def setup(self):
        self.action = LineAction(
                        kp_ang=8.0, ki_ang=0.001, kd_ang=3.0, tolerance_ang=0.03,
                        kp_lin=350.0, ki_lin=0.001, kd_lin=2.0, tolerance_lin=0.005,
                        saturation_ang=(6*np.pi/6), max_integral_ang=np.pi/20, integral_fade_ang=0.75,
                        saturation_lin=(350 * (self.limit_dir - self.limit_esq)/2), max_integral_lin=1.0, integral_fade_lin=0.75,
                        line_dist_std_dev=0.03, linear_decay_std_dev=np.pi/30)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, (self.limit_esq, self.y_position), (self.limit_dir, self.y_position))

    def update(self):
        ball_position = (self.field_data.ball.position.x, self.field_data.ball.position.y)
        self.action.set_goal(ball_position)
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
