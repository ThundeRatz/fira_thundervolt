import py_trees
import numpy as np
import logging

from .core import data
from .core.data import FieldData
from .core.command import TeamCommand
from .behavior_trees import trees


class Coach(object):
    def __init__(self, field_data: FieldData, team_command: TeamCommand):
        self.field_data = field_data
        self.team_command = team_command


    def setup(self):
        self.bb_client = py_trees.blackboard.Client()
        self.bb_client.register_key(key="/goalkeeper/robot_id", access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(key="/defender/robot_id", access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(key="/striker/robot_id", access=py_trees.common.Access.WRITE)

        self.goalkeeper_id = 0
        self.defender_id = 1
        self.striker_id = 2

        self.bb_client.goalkeeper.robot_id = 0
        self.bb_client.defender.robot_id = 1
        self.bb_client.striker.robot_id = 2


    def initialise(self):
        """
        Calculate robots roles when the game starts
        """

        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        goal_pos = np.array((-data.FIELD_LENGTH / 2, 0))

        closer_to_ball = self._ordered_closest_robots_to_point(ball_pos)
        closer_to_goal = self._ordered_closest_robots_to_point(goal_pos)

        self.striker_id = closer_to_ball[0]

        if closer_to_goal[0] != self.striker_id:
            self.goalkeeper_id = closer_to_goal[0]
        else:
            self.goalkeeper_id = closer_to_goal[1]

        self.defender_id = closer_to_ball[1]
        if self.defender_id == self.goalkeeper_id:
            self.defender_id = closer_to_ball[2]

        self.bb_client.goalkeeper.robot_id = self.goalkeeper_id
        self.bb_client.defender.robot_id = self.defender_id
        self.bb_client.striker.robot_id = self.striker_id

        self._create_trees()

        logging.info(f"Goalkeeper: {self.goalkeeper_id}")
        logging.info(f"Defender: {self.defender_id}")
        logging.info(f"Striker: {self.striker_id}")


    def initialise_penalti_defense(self):
        """
        Calculate robots roles when the game starts
        """

        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        goal_pos = np.array((-data.FIELD_LENGTH / 2, 0))

        closer_to_ball = self._ordered_closest_robots_to_point(ball_pos)
        closer_to_goal = self._ordered_closest_robots_to_point(goal_pos)

        self.goalkeeper_id = closer_to_goal[0]

        if closer_to_ball[0] != self.goalkeeper_id:
            self.striker_id = closer_to_ball[0]
        else:
            self.striker_id = closer_to_ball[1]

        self.defender_id = closer_to_goal[1]
        if self.defender_id == self.striker_id:
            self.defender_id = closer_to_goal[2]

        self.bb_client.goalkeeper.robot_id = self.goalkeeper_id
        self.bb_client.defender.robot_id = self.defender_id
        self.bb_client.striker.robot_id = self.striker_id

        self._create_trees()

        logging.info(f"Goalkeeper: {self.goalkeeper_id}")
        logging.info(f"Defender: {self.defender_id}")
        logging.info(f"Striker: {self.striker_id}")


    def update(self):
        if self._defender_striker_swap_condition():
            self._create_trees()

        self.goalkeeper_bt.tick_once()
        self.defender_bt.tick_once()
        self.striker_bt.tick_once()


    def _ordered_closest_robots_to_point(self, point: np.ndarray(2)) -> tuple:
        distance = []
        indexes = list(range(3))
        for robot in self.field_data.robots:
            robot_position = np.array([robot.position.x, robot.position.y])
            distance.append(np.linalg.norm(point - robot_position))

        indexes.sort(key=lambda i: distance[i])

        return tuple(indexes)


    def _defender_striker_swap_condition(self):
        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        defender_pos = np.array((self.field_data.robots[self.defender_id].position.x, self.field_data.robots[self.defender_id].position.y))
        striker_pos = np.array((self.field_data.robots[self.striker_id].position.x, self.field_data.robots[self.striker_id].position.y))

        swap = False
        # First swap condition
        if (ball_pos[0] - defender_pos[0] > data.ROBOT_SIZE / 2) and (defender_pos[0] - striker_pos[0] > data.ROBOT_SIZE / 2) \
                and (abs(ball_pos[1] - defender_pos[1]) < data.ROBOT_SIZE):
            swap = True

        # Second swap condition
        if 0 < ball_pos[0] < data.FIELD_LENGTH / 2 - data.GOAL_AREA_DEPTH * 1.25:
            if (striker_pos[0] - ball_pos[0] > data.ROBOT_SIZE / 2) and (ball_pos[0] - defender_pos[0] > data.ROBOT_SIZE / 2) \
                    and (abs(ball_pos[1] - defender_pos[1]) < data.ROBOT_SIZE):
                swap = True

        if ball_pos[0] > data.FIELD_LENGTH / 8:
            if (ball_pos[0] - defender_pos[0] > data.ROBOT_SIZE / 2) and (abs(striker_pos[1]) > data.GOAL_AREA_WIDTH / 2) and \
                    (abs(defender_pos[1]) < data.GOAL_WIDTH / 2) and (abs(ball_pos[1] < data.GOAL_WIDTH / 2)) \
                    and (abs(ball_pos[1] - defender_pos[1]) < data.ROBOT_SIZE):
                swap = True

        if (striker_pos[0] - defender_pos[0] > data.ROBOT_SIZE / 2) and (defender_pos[0] - ball_pos[0] > data.ROBOT_SIZE / 2):
            swap = True

        if swap:
            logging.info(f"Swap!")
            (self.defender_id, self.striker_id) = (self.striker_id, self.defender_id)
            self.bb_client.defender.robot_id = self.defender_id
            self.bb_client.striker.robot_id = self.striker_id
            return True

        return False


    def _create_trees(self):
        self.goalkeeper_bt = trees.create_goalkeeper_tree(self.field_data, self.team_command)
        self.defender_bt = trees.create_defender_tree(self.field_data, self.team_command)
        self.striker_bt = trees.create_striker_tree(self.field_data, self.team_command)

        self.goalkeeper_bt.setup_with_descendants()
        self.defender_bt.setup_with_descendants()
        self.striker_bt.setup_with_descendants()
