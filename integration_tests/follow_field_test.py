import test_base
import numpy as np

from thundervolt.comm.vision import FiraVision
from thundervolt.comm.control import FiraControl
from thundervolt.actions.follow_field_action import FollowFieldAction
from thundervolt.vector_fields import fields
from thundervolt.core.utils import assert_angle

TEST_ROBOT = 2


def main():
    team_color_yellow = False
    vision = FiraVision(team_color_yellow)
    blue_control = FiraControl(team_color_yellow)

    action = FollowFieldAction(kp_lin=100.0, ki_lin=0.0, kd_lin=1.0, tolerance_lin=0.01, kp_ang=10.0, ki_ang=0.0, kd_ang=3.0, tolerance_ang=0.05)# mudar

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

    repell_field = fields.RadialField(
        target = (-0.3,0),
        max_radius = 0.6,
        decay_radius = 0.1,
        repelling = True,
    )

    attract_field = fields.RadialField(
        target = (0.3,0),
        max_radius = 3.0,
        decay_radius = 0.6,
        repelling = False,
    )

    my_field = fields.VectorField()
    my_field.add(repell_field)
    my_field.add(attract_field)

    vision_data = vision.receive_field_data()
    ball_x = vision_data.ball.position.x
    ball_y = vision_data.ball.position.y

    action.initialize(TEST_ROBOT, my_field)
    action.set_goal(np.array([ball_x, ball_y]))

    while True:
        vision_data = vision.receive_field_data()

        attract_field.target = (vision_data.ball.position.x, vision_data.ball.position.y)
        repell_field.target = (vision_data.foes[0].position.x, vision_data.foes[0].position.y)

        robot_cmd, action_state = action.update(vision_data)

        blue_control.transmit_robot(TEST_ROBOT, robot_cmd.left_speed, robot_cmd.right_speed)

        if action_state == True:
            print("Next goal")
            ball_x = vision_data.ball.position.x
            ball_y = vision_data.ball.position.y
            action.set_goal(np.array([ball_x, ball_y]))

    blue_control.transmit_robot(TEST_ROBOT, 0, 0)

if __name__ == '__main__':
    main()
