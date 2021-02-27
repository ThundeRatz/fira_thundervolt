import test_base  # pylint: disable=import-error
import logging

from thundervolt.comm.control import FiraControlThread
from thundervolt.core.command import TeamCommand

import time
import logging

TEST_ROBOT = 0
WAIT_TIME = 0.5

def main():
    team_command = TeamCommand()

    blue_control = FiraControlThread(team_color_yellow=False, team_command=team_command)

    team_command.commands[TEST_ROBOT].left_speed = -10
    team_command.commands[TEST_ROBOT].right_speed = 10

    logging.debug("--- Starting ---\r\n")
    blue_control.start()

    time.sleep(WAIT_TIME)

    logging.debug("--- Pause ---\r\n")
    blue_control.pause()

    time.sleep(WAIT_TIME)

    logging.debug("--- Resume ---\r\n")
    blue_control.resume()

    time.sleep(WAIT_TIME)

    logging.debug("--- Stoping ---\r\n")
    blue_control.stop()


if __name__ == '__main__':
    main()
