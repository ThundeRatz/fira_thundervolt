import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.vector_fields import fields, combinations
from thundervolt.actions.follow_field_action import FollowFieldAction

class GoBack(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, goal_x):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.goal_x = goal_x

    def setup(self):
        self.action = FollowFieldAction(kp_ang=10.0, ki_ang=0.0, kd_ang=3.0, kp_lin=50.0, ki_lin=0.0,
                                        kd_lin=3.0, tolerance_lin=0.15, base_speed=20,
                                        goal=([self.goal_x, 0]), use_front=True)

        # Field declarations
        attract_field = fields.RadialField(
            target = (self.goal_x, 0),
            max_radius = 3.0,
            decay_radius = 0.3,
            repelling = False,
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.3,
            decay_radius = 0.05,
            multiplier = 1,
        )

        self.my_field = fields.VectorField()
        self.my_field.add(repell_field)
        self.my_field.add(attract_field)

    def initialise(self):
        self.action.initialize(self.parameters.robot_id, self.my_field)

    def update(self):
        self.action.set_goal(np.array([self.goal_x, 0]))
        self.my_field.update(self.field_data, self.parameters.robot_id)
        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass
