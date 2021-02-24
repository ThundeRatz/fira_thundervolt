import test_base  # pylint: disable=import-error
import numpy as np
import py_trees

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.wait_to_get_ball import WaitToGetBall


def main():
    team_color_yellow = False

    field_data = FieldData()
    team_command = TeamCommand()

    vision = FiraVision(team_color_yellow, field_data)
    blue_control = FiraControl(team_color_yellow, team_command)

    blue_control.stop_team()

    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/striker/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.striker.robot_id = 1

    my_tree = WaitToGetBall("Test Node", "/striker", field_data, team_command, 0)

    my_tree.setup()

    try:
        while True:
            vision.update()
            my_tree.tick_once()
            blue_control.update()

    except KeyboardInterrupt:
        blue_control.stop_team()
        my_tree.plot_field()


if __name__ == '__main__':
    main()
