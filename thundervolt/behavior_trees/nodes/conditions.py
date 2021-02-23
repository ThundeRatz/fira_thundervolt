import py_trees
import numpy as np

from .execution_node import ExecutionNode

OUR_GOAL_X = -0.75

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
    pass


class xBallLTd(ExecutionNode):
    pass


class BallDistToGoalLTd(ExecutionNode):
    def __init__(self, name, role, field_data, max_distance):
        super().__init__(name, role, field_data)
        self.max_distance = max_distance

    def update(self):
        if self.field_data.ball.position.x - OUR_GOAL_X < self.max_distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class FoeCloseToBall(ExecutionNode):
    pass


class GoalKeeperOutsideGoal(ExecutionNode):
    pass


class BallDistToPlayerLTd(ExecutionNode):
    def __init__(self, name, role, field_data, distance):
        super().__init__(name, role, field_data)
        self.distance = distance

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        if np.linalg.norm(ball_pos-player_pos) < self.distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
