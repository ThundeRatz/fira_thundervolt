import test_base  # pylint: disable=import-error

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.actions.look_at_action import LookAtAction
import numpy as np

TEST_ROBOT = 1


def main():
    team_color_yellow = False
    vision = FiraVision(team_color_yellow)
    blue_control = FiraControl(team_color_yellow)

    action = LookAtAction(kp=10.0, ki=0.01, kd=1.0, tolerance=0.02)

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

    action.initialize(TEST_ROBOT, (np.pi)/2)

    while True:
        vision_data = vision.receive_field_data()
        robot_cmd, action_state = action.update(vision_data)

        blue_control.transmit_robot(TEST_ROBOT, robot_cmd.left_speed, robot_cmd.right_speed)

        if action_state == True:
            print("Yay finished!\r\n")
            break

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

if __name__ == '__main__':
    main()
