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
        self.goalkeeper_bt = trees.create_goalkeeper_tree(field_data, team_command)
        self.defender_bt = trees.create_defender_tree(field_data, team_command)
        self.striker_bt = trees.create_striker_tree(field_data, team_command)


    def setup(self):
        self.bb_client = py_trees.blackboard.Client()
        self.bb_client.register_key(key="/goalkeeper/robot_id", access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(key="/defender/robot_id", access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(key="/striker/robot_id", access=py_trees.common.Access.WRITE)

        self.bb_client.goalkeeper.robot_id = 0
        self.bb_client.defender.robot_id = 1
        self.bb_client.striker.robot_id = 2

        self.goalkeeper_bt.setup_with_descendants()
        self.defender_bt.setup_with_descendants()
        self.striker_bt.setup_with_descendants()


    def initialise(self):
        """
        Calculate robots roles when the game starts
        """

        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        goal_pos = np.array((-data.FIELD_LENGTH / 2, 0))

        closer_to_ball = self._ordered_closest_robots_to_point(ball_pos)
        closer_to_goal = self._ordered_closest_robots_to_point(goal_pos)

        striker_id = closer_to_ball[0]

        if closer_to_goal[0] != striker_id:
            goalkeeper_id = closer_to_goal[0]
        else:
            goalkeeper_id = closer_to_goal[1]

        defender_id = closer_to_ball[1]
        if defender_id == goalkeeper_id:
            defender_id = closer_to_ball[2]

        self.bb_client.goalkeeper.robot_id = goalkeeper_id
        self.bb_client.defender.robot_id = defender_id
        self.bb_client.striker.robot_id = striker_id

        logging.info(f"Goalkeeper: {goalkeeper_id}")
        logging.info(f"Defender: {defender_id}")
        logging.info(f"Striker: {striker_id}")


    def update(self):
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
        # TODO: espécificar as condições de troca

        defender_id = self.bb_client.defender.robot_id
        striker_id = self.bb_client.striker.robot_id

        ball_pos = np.array((self.field_data.ball.position.x, self.field_data.ball.position.y))
        defender_pos = np.array((self.field_data.robots[defender_id].position.x, self.field_data.robots[defender_id].position.y))
        striker_pos = np.array((self.field_data.robots[striker_id].position.x, self.field_data.robots[striker_id].position.y))

        return False

