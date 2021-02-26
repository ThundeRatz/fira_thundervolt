import test_base  # pylint: disable=import-error
import py_trees

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.trees import create_defender_tree


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

    my_tree = create_defender_tree(field_data, team_command)
    my_tree.setup_with_descendants()

    try:
        while True:
            vision.update()
            print('\nNew Iteration')
            for node in my_tree.tick():
                pass
                # print(node.name)
            print(py_trees.display.ascii_tree(my_tree, show_only_visited=False, show_status=True))
            blue_control.update()
    except KeyboardInterrupt:
        pass

    blue_control.stop_team()

if __name__ == '__main__':
    main()
