import test_base  # pylint: disable=import-error
import numpy as np
import logging

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.actions.line_action import LineAction

TEST_ROBOT = 0

def main():
    team_color_yellow = False
    vision = FiraVision(team_color_yellow)
    blue_control = FiraControl(team_color_yellow)

    action = LineAction(kp_lin=100.0, ki_lin=0.005, kd_lin=1.0, tolerance_lin=0.01, kp_ang=10.0, ki_ang=0.005, kd_ang=3.0, tolerance_ang=0.05)

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

    action.initialize(TEST_ROBOT, np.array([-0.70, -0.18]), np.array([-0.70, 0.18]))
    #action.initialize(TEST_ROBOT, np.array([0.0, -0.5]), np.array([0.0, 0.5]))

    vision_data = vision.receive_field_data()

    goal = np.zeros(2)
    goal[0] = vision_data.ball.position.x
    goal[1] = vision_data.ball.position.y
    action.set_goal(goal)

    while True:
        vision_data = vision.receive_field_data()
        robot_cmd, action_state = action.update(vision_data)

        if action_state == True:
            goal[0] = vision_data.ball.position.x
            goal[1] = vision_data.ball.position.y
            action.set_goal(goal)
            logging.debug("Next Goal")

        blue_control.transmit_robot(TEST_ROBOT, robot_cmd.left_speed, robot_cmd.right_speed)

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

if __name__ == '__main__':
    main()
