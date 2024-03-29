import test_base  # pylint: disable=import-error

from thundervolt.comm.control import FiraControl
from thundervolt.core.command import TeamCommand

import time


def main():
    team_command = TeamCommand()

    blue_control = FiraControl(team_color_yellow=False, team_command=team_command)
    yellow_control = FiraControl(team_color_yellow=True, team_command=team_command)

    current_time = time.time()

    team_command.commands[0].left_speed = -10
    team_command.commands[0].right_speed = 10

    while (time.time() - current_time < 5):
        blue_control.update()
        yellow_control.update()

    current_time = time.time()

    team_command.commands[1].left_speed = 5
    team_command.commands[1].right_speed = 5

    while (time.time() - current_time < 5):
        blue_control.update()
        yellow_control.update()

    current_time = time.time()

    team_command.commands[2].left_speed = -15
    team_command.commands[2].right_speed = -15

    while (time.time() - current_time < 5):
        blue_control.update()
        yellow_control.update()

    team_command.reset()

    blue_control.update()
    yellow_control.update()


if __name__ == '__main__':
    main()
