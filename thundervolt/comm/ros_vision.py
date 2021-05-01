import logging
import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

from ..core.data import FieldData, EntityData
from ..core.utils import assert_angle

BALL_MODEL_NAME = "vss_ball"

BLUE_MODELS_NAMES = ["blue_team/robot_0",
                     "blue_team/robot_1",
                     "blue_team/robot_2"
                    ]

YELLOW_MODELS_NAMES = ["yellow_team/robot_0",
                       "yellow_team/robot_1",
                       "yellow_team/robot_2"
                      ]

class TraveSimVision():
    def __init__(self, team_color_yellow: bool, field_data: FieldData = None):
        self.team_color_yellow = team_color_yellow
        self.field_data = field_data

        self.world_state = ModelStates()

        models_name = ["vss_ball",
                       "yellow_team/robot_0",
                       "yellow_team/robot_1",
                       "yellow_team/robot_2",
                       "blue_team/robot_0",
                       "blue_team/robot_1",
                       "blue_team/robot_2",
                      ]

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=1)


    def callback(self, data: ModelStates):
        self.world_state = data


    def get_world_state(self):
        return self.world_state


    def receive_field_data(self) -> FieldData:
        rcv_field_data = FieldData()
        self._field_data_from_model_states(rcv_field_data, self.world_state)

        return rcv_field_data


    def update(self):
        """
        Update the field_data passed in the constructor
        """
        if self.field_data is None:
            logging.error('FieldData not instantiated', exc_info=True)
        else:
            self._field_data_from_model_states(self.field_data, self.world_state)


    def _quaternion_to_theta(self, q0, q1, q2, q3):
        theta = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        return theta


    def _entity_from_pose_twist(self, entity_data: EntityData, data_pose: Pose, data_twist: Twist, rotate_field=False):
        multiplier = 1 if rotate_field is False else -1
        sum_to_angle = 0 if rotate_field is False else np.pi

        entity_data.position.x = data_pose.position.x * multiplier
        entity_data.position.y = data_pose.position.y * multiplier

        base_theta = self._quaternion_to_theta(data_pose.orientation.w, data_pose.orientation.x,
                                               data_pose.orientation.y, data_pose.orientation.z)
        entity_data.position.theta = assert_angle(base_theta + sum_to_angle)

        entity_data.velocity.x = data_twist.linear.x * multiplier
        entity_data.velocity.y = data_twist.linear.y * multiplier

        entity_data.velocity.theta = data_twist.angular.z


    def _field_data_from_model_states(self, field_data: FieldData, model_states: ModelStates):
        if self.team_color_yellow == True:
            team_models_names = YELLOW_MODELS_NAMES
            foes_models_names = BLUE_MODELS_NAMES
            rotate_field = False
        else:
            team_models_names = BLUE_MODELS_NAMES
            foes_models_names = YELLOW_MODELS_NAMES
            rotate_field = True

        for i, model_name in enumerate(model_states.name):
            if model_name == BALL_MODEL_NAME:
                self._entity_from_pose_twist(field_data.ball, model_states.pose[i], model_states.twist[i], rotate_field)

            if model_name in team_models_names:
                self._entity_from_pose_twist(field_data.robots[int(model_name[-1])], model_states.pose[i], model_states.twist[i], rotate_field)

            if model_name in foes_models_names:
                self._entity_from_pose_twist(field_data.foes[int(model_name[-1])], model_states.pose[i], model_states.twist[i], rotate_field)





