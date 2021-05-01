#!/usr/bin/env python3

import rospy
import logging
import py_trees
import numpy as np

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))

from thundervolt.comm.ros_vision import TraveSimVision
from thundervolt.comm.ros_control import TraveSimControl
from thundervolt.core.data import FieldData
from thundervolt.core.command import TeamCommand
from thundervolt.behavior_trees.nodes.action.look_at_ball import LookAtBall

def main():
    rospy.init_node('line_action', anonymous=True)
    rate = rospy.Rate(60)

    team_color_yellow = True

    field_data = FieldData()
    team_command = TeamCommand()

    vision = TraveSimVision(team_color_yellow, field_data)
    blue_control = TraveSimControl(team_color_yellow, team_command)

    blue_control.stop_team()

    bb_client = py_trees.blackboard.Client()
    bb_client.register_key(key="/goalkeeper/robot_id", access=py_trees.common.Access.WRITE)
    bb_client.goalkeeper.robot_id = 0

    my_tree = LookAtBall("Test Node", "/goalkeeper", field_data, team_command)

    my_tree.setup()

    try:
        while not rospy.is_shutdown():
            vision.update()
            for node in my_tree.tick():
                pass
            blue_control.update()

            rate.sleep()

    except KeyboardInterrupt:
        team_command.reset()

    blue_control.stop_team()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
