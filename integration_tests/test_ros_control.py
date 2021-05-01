#!/usr/bin/env python3

import rospy
import time

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))

from thundervolt.comm.ros_control import TraveSimControl

def main():
    rospy.init_node('robot_controller', anonymous=True)

    yellow_control = TraveSimControl(team_color_yellow=True)

    current_time = time.time()

    while (time.time() - current_time < 5):
        yellow_control.transmit_robot(0, 10, -10)

    current_time = time.time()

    while (time.time() - current_time < 5):
        yellow_control.transmit_robot(1, 5, 5)

    current_time = time.time()

    while (time.time() - current_time < 5):
        yellow_control.transmit_robot(2, -15, -15)

    [yellow_control.transmit_robot(i, 0, 0) for i in range(3)]


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
