import test_base  # pylint: disable=import-error
import py_trees
import logging

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.save import SaveGoal
from thundervolt.core import data

LINE_X = data.FIELD_LENGTH/2 - 0.8 * data.ROBOT_SIZE

def main():
    team_color_yellow = False

    field_data = FieldData()
    team_command = TeamCommand()
    vision = FiraVision(team_color_yellow, field_data)
    blue_control = FiraControl(team_color_yellow, team_command)

    blue_control.stop_team()

    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/goalkeeper/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.goalkeeper.robot_id = 0

    my_tree = SaveGoal("Test Node", "/goalkeeper", field_data, team_command, 3.0, LINE_X, data.GOAL_WIDTH/2)
    my_tree.setup()

    try:
        while True:
            vision.update()
            my_tree.tick_once()
            if my_tree.status == py_trees.common.Status.SUCCESS:
                logging.info('Success!!')
            elif my_tree.status == py_trees.common.Status.FAILURE:
                blue_control.stop_team()
            else:
                blue_control.update()
    except KeyboardInterrupt:
        pass

    blue_control.stop_team()

if __name__ == '__main__':
    main()
