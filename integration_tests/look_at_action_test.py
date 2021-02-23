import test_base  # pylint: disable=import-error
import numpy as np
import time
import logging

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.actions.look_at_action import LookAtAction

TEST_ROBOT = 1

ANGLES = [np.pi / 2, np.pi, 3 * np.pi / 4, 3 * np.pi / 2, 0]

def main():
    team_color_yellow = False
    vision = FiraVision(team_color_yellow)
    blue_control = FiraControl(team_color_yellow)

    action = LookAtAction(kp=10.0, ki=0.005, kd=3.0, tolerance=0.08)

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

    for angle in ANGLES:
        action.initialize(TEST_ROBOT)
        action.set_angle(angle)
        while True:
            vision_data = vision.receive_field_data()
            robot_cmd, action_state = action.update(vision_data)

            blue_control.transmit_robot(TEST_ROBOT, robot_cmd.left_speed, robot_cmd.right_speed)

            if action_state == True:
                logging.debug(f"Reached {angle * 180 / np.pi}º")
                break

        time.sleep(1)

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)
    logging.debug("Yay finished!\r\n")

if __name__ == '__main__':
    main()
