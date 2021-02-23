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


class BallDistanceToDefenseAreaLTd(ExecutionNode):
    def __init__(self, name, role, field_data, distance):
        super().__init__(name, role, field_data)
        self.distance = distance

    def update(self):
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        dist_to_area = 0.0

        if ball_pos[0] < (-data.FIELD_LENGTH + data.GOAL_AREA_DEPTH):
            if abs(ball_pos[1]) > data.GOAL_AREA_WIDTH / 2:
                dist_to_area = abs(ball_pos[1]) - data.GOAL_AREA_WIDTH / 2
        else:
            vertical_coordinate = np.clip (ball_pos[1], -data.GOAL_AREA_WIDTH / 2, data.GOAL_AREA_WIDTH / 2)
            closest_point = np.array([data.FIELD_LENGTH / 2 - data.GOAL_AREA_DEPTH, vertical_coordinate])
            dist_to_area = np.linalg.norm(closest_point - ball_pos)

        if dist_to_area < self.distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class PlayerDistanceToDefenseAreaLTd(ExecutionNode):
    def __init__(self, name, role, field_data, distance):
        super().__init__(name, role, field_data)
        self.distance = distance

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y
        dist_to_area = 0.0

        if player_pos[0] < (-data.FIELD_LENGTH + data.GOAL_AREA_DEPTH):
            if abs(player_pos[1]) > data.GOAL_AREA_WIDTH / 2:
                dist_to_area = abs(player_pos[1]) - data.GOAL_AREA_WIDTH / 2
        else:
            vertical_coordinate = np.clip (player_pos[1], -data.GOAL_AREA_WIDTH / 2, data.GOAL_AREA_WIDTH / 2)
            closest_point = np.array([data.FIELD_LENGTH / 2 - data.GOAL_AREA_DEPTH, vertical_coordinate])
            dist_to_area = np.linalg.norm(closest_point - player_pos)

        if dist_to_area < self.distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


