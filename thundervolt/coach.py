import py_trees
import numpy as np

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

    def update(self):
        # Test update
        self.team_command.commands[0].left_speed += 0.1
        self.team_command.commands[0].right_speed += 0.1

        print(self.team_command)

    def reset(self):
        # Reseta as posições dos jogadores
        pass
