from .receiver import Receiver
from .protocols import packet_pb2
from ..core.data import FieldData, EntityData
from ..core.math import assert_angle

import json
from google.protobuf.json_format import MessageToJson
import numpy as np

class FiraVision(Receiver):
    def __init__(self, team_color_yellow=False, vision_ip='224.0.0.1', vision_port=10002):
        super(FiraVision, self).__init__(vision_ip, vision_port)

        self.team_color_yellow = team_color_yellow


    def receive(self):
        data = super().receive()
        return data


    def receive_dict(self):
        """
        Receive packet and decode.
        """

        data = self.receive()
        decoded_data = packet_pb2.Environment().FromString(data) # pylint: disable=no-member
        vision_frame = decoded_data.frame

        vision_data = json.loads(MessageToJson(vision_frame))

        return vision_data


    def receive_field_data(self) -> FieldData:
        vision_data_dict = self.receive_dict()

        field_data = FieldData()
        self._field_data_from_dict(field_data, vision_data_dict)

        return field_data


    def _entity_from_dict(self, entity_data: EntityData, data_dict, rotate_field=False):
        multiplier = 1 if rotate_field is False else -1
        sum_to_angle = 0 if rotate_field is False else np.pi

        entity_data.position.x = data_dict.get('x', 0) * multiplier
        entity_data.position.y = data_dict.get('y', 0) * multiplier

        # The ball dict does not contain 'orientation' so it will always be 0
        entity_data.position.theta = assert_angle(data_dict.get('orientation', 0) + sum_to_angle)

        entity_data.velocity.x = data_dict.get('vx', 0) * multiplier
        entity_data.velocity.y = data_dict.get('vy', 0) * multiplier

        # The ball dict does not contain 'vorientation' so it will always be 0
        entity_data.velocity.theta = data_dict.get('vorientation', 0)


    def _field_data_from_dict(self, field_data: FieldData, raw_data_dict):
        if self.team_color_yellow == True:
            team_list_of_dicts = raw_data_dict.get('robotsYellow')
            foes_list_of_dicts = raw_data_dict.get('robotsBlue')
            rotate_field = True
        else:
            team_list_of_dicts = raw_data_dict.get('robotsBlue')
            foes_list_of_dicts = raw_data_dict.get('robotsYellow')
            rotate_field = False

        if 'ball' in raw_data_dict:
            self._entity_from_dict(field_data.ball, raw_data_dict['ball'], rotate_field)

        for i in range(len(team_list_of_dicts)):
            self._entity_from_dict(field_data.robots[i], team_list_of_dicts[i], rotate_field)

        for i in range(len(foes_list_of_dicts)):
            self._entity_from_dict(field_data.foes[i], foes_list_of_dicts[i], rotate_field)
