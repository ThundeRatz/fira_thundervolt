import test_base  # pylint: disable=import-error

from thundervolt.comm.control import FiraControl
from thundervolt.core.command import TeamCommand

import time


def main():
    blue_control = FiraControl(team_color_yellow=False)
    yellow_control = FiraControl()

    team_command = TeamCommand()

    current_time = time.time()

    team_command.commands[0].left_speed = -10
    team_command.commands[0].right_speed = 10

    while (time.time() - current_time < 5):
        blue_control.transmit_team(team_command)
        yellow_control.transmit_team(team_command)

    current_time = time.time()

    team_command.commands[1].left_speed = 5
    team_command.commands[1].right_speed = 5

    while (time.time() - current_time < 5):
        blue_control.transmit_team(team_command)
        yellow_control.transmit_team(team_command)

    current_time = time.time()

    team_command.commands[2].left_speed = -15
    team_command.commands[2].right_speed = -15

    while (time.time() - current_time < 5):
        blue_control.transmit_team(team_command)
        yellow_control.transmit_team(team_command)

    team_command.reset()

    blue_control.transmit_team(team_command)
    yellow_control.transmit_team(team_command)


if __name__ == '__main__':
    main()
