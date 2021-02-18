import numpy as np
from .math import assert_angle

FIELD_WIDTH = 1.3
FIELD_LENGTH = 1.5

GOAL_WIDTH = 0.4
GOAL_DEPTH = 0.1


class Pose2D:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f'x: {self.x:.02f}  y: {self.y:.02f}  th: {self.theta:.02f}'

    def __repr__(self):
        return f'Pose2D({self})'

class EntityData:
    def __init__(self, position: Pose2D = Pose2D(), velocity: Pose2D = Pose2D()):
        self.position = position
        self.velocity = velocity

    def __str__(self):
        msg = (
            f'Position: {self.position}\n'
            f'Velocity: {self.velocity}\n'
        )
        return msg

    def __repr__(self):
        return f'EntityData({self})'

    def _from_dict(self, data_dict, rotate_field=False):
        multiplier = 1 if rotate_field is False else -1
        sum_to_angle = 0 if rotate_field is False else np.pi

        self.position.x = data_dict.get('x', 0) * multiplier
        self.position.y = data_dict.get('y', 0) * multiplier

        # The ball dict does not contain 'orientation' so it will always be 0
        self.position.theta = assert_angle(data_dict.get('orientation', 0) + sum_to_angle)

        self.velocity.x = data_dict.get('vx', 0) * multiplier
        self.velocity.y = data_dict.get('vy', 0) * multiplier

        # The ball dict does not contain 'vorientation' so it will always be 0
        self.velocity.theta = data_dict.get('vorientation', 0)

class FieldData:
    def __init__(self, team_color='blue'):
        self.robots = [EntityData() for i in range(3)]
        self.foes = [EntityData() for i in range(3)]
        self.ball = EntityData()

        self.team_color = team_color
        self.foes_team_color = 'yellow' if self.team_color == 'blue' else 'blue'

    def __str__(self):
        msg = f'BALL\n{self.ball}'
        for i in range(3):
            msg += f'\nROBOT_{i}\n{self.robots[i]}'
        for i in range(3):
            msg += f'\nFOE_{i}\n{self.foes[i]}'
        return msg

    def __repr__(self):
        return f'FieldData({self})'

    def from_vision_raw(self, raw_data_dict):
        if self.team_color == 'yellow':
            team_list_of_dicts = raw_data_dict.get('robotsYellow')
            foes_list_of_dicts = raw_data_dict.get('robotsBlue')
            rotate_field = True
        else:
            team_list_of_dicts = raw_data_dict.get('robotsBlue')
            foes_list_of_dicts = raw_data_dict.get('robotsYellow')
            rotate_field = False

        if 'ball' in raw_data_dict:
            self.ball._from_dict(raw_data_dict['ball'], rotate_field)

        # self.ball.position.x = -0.375
        # self.ball.position.theta = 0

        # print(self.ball)
        # print(self.robots[0])

        # self.robots[0].velocity.y = 2.29e-05
        # self.robots[0].velocity.theta = 4.70

        # print(self.ball)
        # print(self.robots[0])

        for i in range(len(team_list_of_dicts)):
            self.robots[i]._from_dict(team_list_of_dicts[i], rotate_field)

        for i in range(len(foes_list_of_dicts)):
            self.foes[i]._from_dict(foes_list_of_dicts[i], rotate_field)
