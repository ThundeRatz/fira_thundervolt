import py_trees
import numpy as np

from .execution_node import ExecutionNode
from thundervolt.core import data, utils

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
    def __init__(self, name, role, field_data, player_tolerance, goal_tolerance):
        super().__init__(name, role, field_data)

        self.player_tol = player_tolerance
        self.goal_tol = goal_tolerance
        self.lpost = np.array((data.FIELD_LENGTH/2, -data.GOAL_WIDTH/2))
        self.rpost = np.array((data.FIELD_LENGTH/2, data.GOAL_WIDTH/2))

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y
        player_theta = utils.assert_half_angle(self.field_data.robots[self.parameters.robot_id].theta)
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        player_ball = ball_pos - player_pos
        player_lpost = self.lpost - player_pos
        player_rpost = self.rpost - player_pos

        angle_ball = utils.vectors_angle(player_ball)
        angle_lpost = utils.vectors_angle(player_lpost)
        angle_rpost = utils.vectors_angle(player_rpost)

        if ((player_theta-self.player_tol <= angle_ball <= player_theta+self.player_tol)
        and (angle_lpost-self.goal_tol <= angle_ball <= angle_rpost+self.goal_tol)):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


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
    def __init__(self, name, role, field_data, d_position):
        super().__init__(name, role, field_data)
        self.d_position = d_position

    def update(self):
        if self.field_data.ball.position.x < self.d_position:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    pass

# Temporary condition node
class yBallLTd(ExecutionNode):
    def __init__(self, name, role, field_data, d_position):
        super().__init__(name, role, field_data)
        self.d_position = d_position

    def update(self):
        if self.field_data.ball.position.y < self.d_position:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    pass


class BallDistToGoalLTd(ExecutionNode):
    def __init__(self, name, role, field_data, max_distance):
        super().__init__(name, role, field_data)
        self.max_distance = max_distance

    def update(self):
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))

        if self.field_data.ball.position.y > data.GOAL_WIDTH/2:
            upper_goalpost = np.array((-data.FIELD_LENGTH/2, data.GOAL_WIDTH/2))
            distance = np.linalg.norm(ball_pos - upper_goalpost)
        elif self.field_data.ball.position.y < -data.GOAL_WIDTH/2:
            lower_goalpost = np.array((-data.FIELD_LENGTH/2, -data.GOAL_WIDTH/2))
            distance = np.linalg.norm(ball_pos - lower_goalpost)
        else:
            distance = self.field_data.ball.position.x - (-data.FIELD_LENGTH/2)

        if distance < self.max_distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class FoeCloserToBall(ExecutionNode):
    def __init__(self, name, role, field_data):
        super().__init__(name, role, field_data)

    def update(self):
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        num_team_robots = 3
        distance = np.zeros(3)

        for i in range(num_team_robots):
            robot_pos = np.array((self.field_data.robots[i].position.x, self.field_data.robots[i].position.y))
            distance[i] = np.linalg.norm(ball_pos-robot_pos)
        min_ally_dist = min(distance)

        for i in range(num_team_robots):
            robot_pos = np.array((self.field_data.foes[i].position.x, self.field_data.foes[i].position.y))
            distance[i] = np.linalg.norm(ball_pos-robot_pos)
        min_foe_dist = min(distance)

        if min_foe_dist <= min_ally_dist:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE



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
        dist_to_area = None

        if ball_pos[0] < (-data.FIELD_LENGTH / 2 + data.GOAL_AREA_DEPTH):
            if abs(ball_pos[1]) > data.GOAL_AREA_WIDTH / 2:
                dist_to_area = abs(ball_pos[1]) - data.GOAL_AREA_WIDTH / 2
        else:
            vertical_coordinate = np.clip(ball_pos[1], -data.GOAL_AREA_WIDTH / 2, data.GOAL_AREA_WIDTH / 2)
            closest_point = np.array([-data.FIELD_LENGTH / 2 + data.GOAL_AREA_DEPTH, vertical_coordinate])
            dist_to_area = np.linalg.norm(closest_point - ball_pos)

        if dist_to_area is None:
            return py_trees.common.Status.SUCCESS

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
        dist_to_area = None

        if player_pos[0] < (-data.FIELD_LENGTH / 2 + data.GOAL_AREA_DEPTH):
            if abs(player_pos[1]) > data.GOAL_AREA_WIDTH / 2:
                dist_to_area = abs(player_pos[1]) - data.GOAL_AREA_WIDTH / 2
        else:
            vertical_coordinate = np.clip(player_pos[1], -data.GOAL_AREA_WIDTH / 2, data.GOAL_AREA_WIDTH / 2)
            closest_point = np.array([-data.FIELD_LENGTH / 2 + data.GOAL_AREA_DEPTH, vertical_coordinate])
            dist_to_area = np.linalg.norm(closest_point - player_pos)

        if dist_to_area is None:
            return py_trees.common.Status.SUCCESS

        if dist_to_area < self.distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class PlayerDistToGoalLTd(ExecutionNode):
    def __init__(self, name, role, field_data, max_distance):
        super().__init__(name, role, field_data)
        self.max_distance = max_distance

    def update(self):
        player_pos = np.zeros(2)
        player_pos[0] = self.field_data.robots[self.parameters.robot_id].position.x
        player_pos[1] = self.field_data.robots[self.parameters.robot_id].position.y

        if player_pos[1] > data.GOAL_WIDTH/2:
            upper_goalpost = np.array((-data.FIELD_LENGTH/2, data.GOAL_WIDTH/2))
            distance = np.linalg.norm(player_pos - upper_goalpost)
        elif player_pos[1] < -data.GOAL_WIDTH/2:
            lower_goalpost = np.array((-data.FIELD_LENGTH/2, -data.GOAL_WIDTH/2))
            distance = np.linalg.norm(player_pos - lower_goalpost)
        else:
            distance = player_pos[0] - (-data.FIELD_LENGTH/2)

        if distance < self.max_distance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
