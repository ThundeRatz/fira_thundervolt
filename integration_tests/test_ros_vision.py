#!/usr/bin/env python3

import rospy
import time

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))

from thundervolt.comm.ros_vision import TraveSimVision


def main():
    rospy.init_node('field_vision', anonymous=True)
    rate = rospy.Rate(1)

    vision_receiver = TraveSimVision(True)

    while not rospy.is_shutdown():
        print(vision_receiver.receive_field_data())
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
