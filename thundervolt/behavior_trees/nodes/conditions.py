import py_trees
import numpy as np

from .execution_node import ExecutionNode
from thundervolt.core import data

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
        if self.field_data.ball.position.x - (-data.FIELD_LENGTH/2) < self.max_distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class FoeCloseToBall(ExecutionNode):
    def __init__(self, name, role, field_data):
        super().__init__(name, role, field_data)

    def update(self):
        ball_pos = np.array((self.field_data.ball.position))
        num_team_robots = 3
        distance = np.zeros(3)

        for i in range(num_team_robots):
            robot_pos = np.array((self.field_data.robots[i].position))
            distance[i] = np.linalg.norm(ball_pos-robot_pos)
        min_ally_dist = min(distance)

        for i in range(num_team_robots):
            robot_pos = np.array((self.field_data.foes[i].position))
            distance[i] = np.linalg.norm(ball_pos-robot_pos)
        min_foe_dist = min(distance)

        if min_foe_dist <= min_ally_dist:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


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
