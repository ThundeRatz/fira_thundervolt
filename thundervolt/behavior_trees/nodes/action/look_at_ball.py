import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.core import utils
from thundervolt.actions.look_at_action import LookAtAction


class LookAtBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command):
        """
        Create an action node to look at the ball

        Args:
            name (str): Behaviour name.
            role (str): Robot role namespace for black board client.
            field_data (FieldData): Field information to be stored in the node.
            team_command (TeamCommand): Angular speed commands for the robots.
        """

        super().__init__(name, role, field_data)
        self.team_command = team_command

    def setup(self):
        self.action = LookAtAction(
                        kp=7.0, ki=0.0, kd=1.5, tolerance=np.pi/10,
                        saturation=7 * np.pi/2, max_integral=np.pi/10, integral_fade=0.5)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id)

    def update(self):
        robot_position = np.array((self.field_data.robots[self.parameters.robot_id].position.x,
                            self.field_data.robots[self.parameters.robot_id].position.y))
        ball_position = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        robot_dir = utils.from_polar(self.field_data.robots[self.parameters.robot_id].position.theta)
        to_ball = ball_position - robot_position

        # Decide the robot side to use
        if abs(utils.vectors_angle(to_ball, robot_dir)) > np.pi/2:
            to_ball *= -1

        self.action.set_angle(np.arctan2(to_ball[1], to_ball[0]))

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
