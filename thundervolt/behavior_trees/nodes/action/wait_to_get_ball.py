import numpy as np
import py_trees

from ..execution_node import ExecutionNode
from thundervolt.core import data
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields import fields, combinations

class WaitToGetBall(ExecutionNode):
    def __init__(self, name, role, field_data, team_command, x_partition):
        super().__init__(name, role, field_data)
        self.team_command = team_command
        self.x_partition = x_partition


    def setup(self):
        self.action = FollowFieldAction(
                        kp_ang=8.0, ki_ang=0.001, kd_ang=3.0, tolerance_ang=0.03,
                        kp_lin=350.0, ki_lin=0.001, kd_lin=2.0, tolerance_lin=0.005,
                        saturation_ang=(6*np.pi/6), max_integral_ang=np.pi/20, integral_fade_ang=0.75,
                        saturation_lin=(350 * (self.limit_sup - self.limit_inf)/2), max_integral_lin=1.0, integral_fade_lin=0.75,
                        line_dist_std_dev=0.03, linear_decay_std_dev=np.pi/30)


    def initialise(self):
        division_field = fields.LineField(
            target = (self.x_partition, 0),
            theta = np.pi / 2,
            size = data.FIELD_WIDTH / 2,
            side = 'positive',
            repelling = True,
        )

        repell_field = combinations.ObstaclesField(
            max_radius = 0.3,
            decay_radius = 0.05,
            multiplier = 1,
        )

        self.target_field = fields.OrientedAttractingField(
            target = (self.field_data.ball.position.x + data.ROBOT_SIZE, self.field_data.ball.position.y),
            direction = (1,0),
            node_radius = data.ROBOT_SIZE
        )

        self.my_field = fields.VectorField()
        self.my_field.add(division_field)
        self.my_field.add(repell_field)
        self.my_field.add(self.target_field)
        self.action.initialize(self.parameters.robot_id, self.my_field)


    def update(self):
        goal = (self.field_data.ball.position.x + data.ROBOT_SIZE, self.field_data.ball.position.y)
        
        self.target_field.target(goal)
        self.my_field.update(self.field_data, self.parameters.robot_id)
        self.action.set_goal(np.array(goal))

        robot_cmd, action_status = self.action.update(self.field_data)
        self.team_command.commands[self.parameters.robot_id] = robot_cmd

        if action_status:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        pass
