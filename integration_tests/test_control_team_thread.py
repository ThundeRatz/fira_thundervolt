import test_base  # pylint: disable=import-error

from thundervolt.comm.control import FiraControlThread
from thundervolt.core.command import TeamCommand

import time

TEST_ROBOT = 0

def main():
    team_command = TeamCommand()

    blue_control = FiraControlThread(team_color_yellow=False, team_command=team_command)

    team_command.commands[TEST_ROBOT].left_speed = -10
    team_command.commands[TEST_ROBOT].right_speed = 10

    print("--- Starting ---\r\n")
    blue_control.start()

    time.sleep(5)

    print("--- Pause ---\r\n")
    blue_control.pause()

    time.sleep(5)

    print("--- Resume ---\r\n")
    blue_control.resume()

    time.sleep(5)

    print("--- Stoping ---\r\n")
    blue_control.stop()


if __name__ == '__main__':
    main()
