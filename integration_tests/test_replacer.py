import test_base  # pylint: disable=import-error
import numpy as np

from thundervolt.comm.replacer import ReplacerComm
from thundervolt.comm.referee import RefereeComm
from thundervolt.core import data
from thundervolt.core.utils import vectors_angle

import time


def main():
    blue_replacer = ReplacerComm(team_color_yellow=False)
    referee = RefereeComm()

    robot_zero = data.EntityData()
    robot_zero.position.x = data.FIELD_LENGTH/4 - data.ROBOT_SIZE * 1.2
    robot_zero.position.y = - data.ROBOT_SIZE * 0.3

    ball_entrypoint = np.array([data.FIELD_LENGTH/2, data.GOAL_WIDTH/2 - data.BALL_RADIUS * 2.6])
    kick_angle = vectors_angle(ball_entrypoint - np.array([robot_zero.position.x, robot_zero.position.y])) * 180 / np.pi
    robot_zero.position.theta = kick_angle

    try:
        while True:
            game_state = referee.receive().get('foul', 'STOP')

            if game_state == 'PENALTY_KICK':
                blue_replacer.place_team([(robot_zero, 1)])
                break

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
