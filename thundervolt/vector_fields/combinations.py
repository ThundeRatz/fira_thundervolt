import numpy as np

from thundervolt.core import data
from thundervolt.vector_fields.fields import VectorField, RadialField, LineField

class WallField(VectorField):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.max_dist = kwargs.get('max_dist', None)
        self.decay_dist = kwargs.get('decay_dist', None)

        self.multiplier = kwargs.get('multiplier', 1)
        self.update_rule = kwargs.get('update_rule', None)

        top_wall = LineField(
            target = (0, data.FIELD_WIDTH / 2),
            theta = 0.0,
            size = data.FIELD_LENGTH / 2,
            side = 'positive',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )

        botton_wall = LineField(
            target = (0, -data.FIELD_WIDTH / 2),
            theta = 0.0,
            size = data.FIELD_LENGTH / 2,
            side = 'negative',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )

        left_top_wall = LineField(
            target = (-data.FIELD_LENGTH / 2, data.FIELD_WIDTH / 2),
            theta = -np.pi / 2,
            size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
            only_forward = True,
            side = 'negative',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )

        left_botton_wall = LineField(
            target = (-data.FIELD_LENGTH / 2, -data.FIELD_WIDTH / 2),
            theta = np.pi / 2,
            size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
            only_forward = True,
            side = 'positive',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )


        right_top_wall = LineField(
            target = (data.FIELD_LENGTH / 2, data.FIELD_WIDTH / 2),
            theta = -np.pi / 2,
            size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
            only_forward = True,
            side = 'positive',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )

        right_botton_wall = LineField(
            target = (data.FIELD_LENGTH / 2, -data.FIELD_WIDTH / 2),
            theta = np.pi / 2,
            size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
            only_forward = True,
            side = 'negative',
            repelling = True,
            max_dist = self.max_dist,
            decay_dist = self.decay_dist,
            multiplier = self.multiplier,
        )

        self.add(top_wall)
        self.add(botton_wall)
        self.add(left_top_wall)
        self.add(left_botton_wall)
        self.add(right_top_wall)
        self.add(right_botton_wall)

    def update(self, field_data, robot_id):
        if callable(self.update_rule):
            self.update_rule(self, field_data, robot_id)

        for field in self.field_childrens:
            field.max_dist = self.max_dist
            field.decay_dist = self.decay_dist
            field.multiplier = self.multiplier


class ObstaclesField(VectorField):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.max_radius = kwargs.get('max_radius', None)
        self.decay_radius = kwargs.get('decay_radius', None)

        self.multiplier = kwargs.get('multiplier', 1)
        self.update_rule = kwargs.get('update_rule', None)

        self.field_childrens = [RadialField(
                                    target=np.zeros(2),
                                    repelling=True,
                                    max_radius=self.max_radius,
                                    decay_radius=self.decay_radius,
                                    multiplier=self.multiplier)
                                for i in range(5)]

    def update(self, field_data, robot_id):
        if callable(self.update_rule):
            self.update_rule(self, field_data, robot_id)

        for field in self.field_childrens:
            field.max_radius = self.max_radius
            field.decay_radius = self.decay_radius
            field.multiplier = self.multiplier

        iterator = 0
        for i in range(3):
            if i == robot_id:
                continue

            self.field_childrens[iterator].target = (field_data.robots[i].position.x, field_data.robots[i].position.y)
            iterator += 1

        for i in range(3):
            self.field_childrens[iterator].target = (field_data.foes[i].position.x, field_data.foes[i].position.y)
            iterator += 1

