import test_base  # pylint: disable=import-error
import numpy as np
import py_trees

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.go_near_corner import GoNearCorner

def main():
    team_color_yellow = False

    field_data = FieldData()
    team_command = TeamCommand()

    vision = FiraVision(team_color_yellow, field_data)
    blue_control = FiraControl(team_color_yellow, team_command)

    blue_control.stop_team()

    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/defender/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.defender.robot_id = 1

    # Tests
    # my_tree = GoNearCorner("Test Node", "/defender", field_data, team_command, upper = False)
    my_tree = GoNearCorner("Test Node", "/defender", field_data, team_command, upper = True)

    my_tree.setup()

    try:
        while True:
            vision.update()
            my_tree.tick_once()
            blue_control.update()
            if my_tree.status == py_trees.common.Status.SUCCESS:
                print("End")
                break

    except KeyboardInterrupt:
        team_command.reset()

    blue_control.stop_team()

if __name__ == '__main__':
    main()
