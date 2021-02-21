from typing import Tuple
import numpy as np

from .action import Action
from thundervolt.core.pid_controller import pidController
from thundervolt.core.utils import from_polar, assert_angle, gaussian
from thundervolt.core.data import FieldData, ROBOT_SIZE
from thundervolt.core.command import RobotCommand


class FollowFieldAction(Action):
    def __init__(self, kp_ang, ki_ang, kd_ang, kp_lin=0.0, ki_lin=0.0, kd_lin=0.0, tolerance_lin=0.15,
                 base_speed=20.0, use_front=True, goal=None, saturation_ang=None, saturation_lin=None):
        """
        Create a follow field action object

        Args:
            kp_ang (float): Proportional constant for angular error
            ki_ang (float): Integrative constant for angular error
            kd_ang (float): Derivative constant for angular error
            kp_lin (float, optional): Proportional constant for angular error. Defaults to 0.0.
            ki_lin (float, optional): Integrative constant for angular error. Defaults to 0.0.
            kd_lin (float, optional): Derivative constant for angular error. Defaults to 0.0.
            tolerance_lin (float, optional): Settling interval around set point. Defaults to 0.15.
            base_speed (float, optional): Base angular speed (rad/s). Defaults to 20.0.
            use_front (bool, optional): Where to compute field vector.
                If true, use robot front, otherwise use robot center. Defaults to True.
            goal (ndarray, optional): Array with goal coordinates. Defaults to None.
            saturation_ang (float, optional): Angular pid controller saturation
            saturation_lin (float, optional): Linear pid controller saturation
        """
        super().__init__()
        self.controller_ang = pidController(kp_ang, ki_ang, kd_ang)
        if saturation_ang is not None:
            self.controller_ang.saturation = saturation_ang
        else:
            self.controller_ang.saturation = np.pi * 2 * kp_ang

        self.tolerance_lin = tolerance_lin
        self.controller_lin = pidController(kp_lin, ki_lin, kd_lin)
        self.controller_lin.saturation = saturation_lin

        self.goal = goal
        self.base_speed = base_speed
        self.use_front = use_front

    def initialize(self, robot_id, vector_field):
        super().initialize(robot_id)
        self.controller_ang.reset()
        self.controller_lin.reset()
        self.vector_field = vector_field

    def set_goal(self, goal):
        self.goal = goal

    def update(self, field_data: FieldData) -> Tuple[RobotCommand, bool]:
        actual_angle = field_data.robots[self.robot_id].position.theta
        looking_direction = from_polar(actual_angle)

        pos_x = field_data.robots[self.robot_id].position.x
        pos_y = field_data.robots[self.robot_id].position.y
        robot_center = np.array((pos_x, pos_y))

        # Check if the action should use the linear controller or the base speed
        if self.goal is not None:
            goal_vector = self.goal - robot_center
            response_lin = - \
                self.controller_lin.update(np.linalg.norm(goal_vector))

            # Check if reached goal
            if np.linalg.norm(goal_vector) < self.tolerance_lin:
                return (RobotCommand(), True)
        else:
            response_lin = self.base_speed

        # Calculate univector with robot center
        univector = self.vector_field.compute(robot_center)
        ang_univector = np.arctan2(univector[1], univector[0])

        # Check if it is better to go forward or backward
        if abs(assert_angle(ang_univector - actual_angle)) > np.pi/2:
            actual_angle = assert_angle(actual_angle + np.pi)
            looking_direction *= -1
            response_lin *= -1

        # Calculate univector again in case you want to use robot front
        if self.use_front:
            front_position = robot_center + \
                (looking_direction * ROBOT_SIZE / 2)
            univector = self.vector_field.compute(front_position)
            ang_univector = np.arctan2(univector[1], univector[0])

        # Calculate angle to the unviector direction
        to_univector_angle = assert_angle(actual_angle - ang_univector)

        # Apply de decay on the linear command based on the error
        response_lin *= gaussian(to_univector_angle, std_dev=np.pi/6)

        response_ang = self.controller_ang.update(to_univector_angle)

        return (RobotCommand(response_lin - response_ang, response_lin + response_ang), False)
