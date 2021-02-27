import test_base  # pylint: disable=import-error
import numpy as np
import py_trees
import logging

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.go_to_foes_goal import GoToFoesGoal


def main():
    team_color_yellow = False

    field_data = FieldData()
    team_command = TeamCommand()

    vision = FiraVision(team_color_yellow, field_data)
    blue_control = FiraControl(team_color_yellow, team_command)

    blue_control.stop_team()

    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/goalkeeper/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.goalkeeper.robot_id = 1

    distance = 0.2
    my_tree = GoToFoesGoal("Test Node", "/goalkeeper", field_data, team_command, distance)

    my_tree.setup()

    try:
        while True:
            vision.update()
            my_tree.tick_once()
            blue_control.update()

            if my_tree.status == py_trees.common.Status.SUCCESS:
                logging.info("Reach goal!")
                break
            if my_tree.status == py_trees.common.Status.FAILURE:
                logging.info("Failure!")
                break

    except KeyboardInterrupt:
        my_tree.plot_field()

    blue_control.stop_team()

if __name__ == '__main__':
    main()
