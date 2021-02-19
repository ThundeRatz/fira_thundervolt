import base_example

import numpy as np

from thundervolt.core import data
from thundervolt.vector_fields import fields, plotter

my_plotter = plotter.FieldPlotter('Walls')

top_wall = fields.LineField(
    target = (0, data.FIELD_WIDTH / 2),
    theta = 0.0,
    size = data.FIELD_LENGTH / 2,
    side = 'positive',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)

botton_wall = fields.LineField(
    target = (0, -data.FIELD_WIDTH / 2),
    theta = 0.0,
    size = data.FIELD_LENGTH / 2,
    side = 'negative',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)

left_top_wall = fields.LineField(
    target = (-data.FIELD_LENGTH / 2, data.FIELD_WIDTH / 2),
    theta = -np.pi / 2,
    size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
    only_forward = True,
    side = 'negative',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)

left_botton_wall = fields.LineField(
    target = (-data.FIELD_LENGTH / 2, -data.FIELD_WIDTH / 2),
    theta = np.pi / 2,
    size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
    only_forward = True,
    side = 'positive',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)


right_top_wall = fields.LineField(
    target = (data.FIELD_LENGTH / 2, data.FIELD_WIDTH / 2),
    theta = -np.pi / 2,
    size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
    only_forward = True,
    side = 'positive',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)

right_botton_wall = fields.LineField(
    target = (data.FIELD_LENGTH / 2, -data.FIELD_WIDTH / 2),
    theta = np.pi / 2,
    size = (data.FIELD_WIDTH - data.GOAL_WIDTH) / 2,
    only_forward = True,
    side = 'negative',
    repelling = True,
    max_dist = 0.15,
    decay_dist = 0.08,
)

my_field = fields.VectorField()
my_field.add(top_wall)
my_field.add(botton_wall)
my_field.add(left_top_wall)
my_field.add(left_botton_wall)
my_field.add(right_top_wall)
my_field.add(right_botton_wall)

my_plotter.plot(my_field)
