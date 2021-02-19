import base_example

import numpy as np

from thundervolt.core import math, data
from thundervolt.vector_fields import fields, plotter

def update_field(field, field_data, robot_id):
    field.max_radius += 0.2
    field.decay_radius += 0.2

field_data = data.FieldData()
my_plotter = plotter.FieldPlotter('Radial Field Update')

updatable_field = fields.RadialField(
    update_rule = update_field,
    target = (0,0),
    max_radius = 0.2,
    decay_radius = 0.0,
    repelling = True,
)

my_field = fields.VectorField()
my_field.add(updatable_field)

print("Close window to see next step")
my_plotter.plot(my_field)

my_field.update(field_data, 0)
my_plotter.plot(my_field)

my_field.update(field_data, 0)
my_plotter.plot(my_field)

