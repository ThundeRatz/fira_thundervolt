import py_trees
import numpy as np

from .execution_node import ExecutionNode

class xPlayerLTd(ExecutionNode):
    def __init__(self, name, role, field_data, d_position):
        super().__init__(name, role, field_data)

        self.d_position = d_position

    def update(self):
        if self.field_data.robots[self.parameters.robot_id].position.x < self.d_position:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GoodStrikerOrientation(ExecutionNode):
    pass


class xPlayerLTxBall(ExecutionNode):
    def __init__(self, name, role, field_data):
        super().__init__(name, role, field_data)

    def update(self):
        player_x = self.field_data.robots[self.parameters.robot_id].position.x
        ball_x = self.field_data.ball.position.x

        if player_x < ball_x:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class xBallLTd(ExecutionNode):
    pass


class BallDistToGoalLTd(ExecutionNode):
    pass


class FoeCloseToBall(ExecutionNode):
    pass


class GoalKeeperOutsideGoal(ExecutionNode):
    pass


class BallDistToPlayerLTd(ExecutionNode):
    pass


class TimeBallReachGoalLTt(ExecutionNode):
    pass
