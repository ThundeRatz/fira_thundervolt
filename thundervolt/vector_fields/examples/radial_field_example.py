import base_example

import numpy as np

from thundervolt.core import math, data
from thundervolt.vector_fields import fields, plotter

field_data = data.FieldData()
my_plotter = plotter.FieldPlotter('Radial Field')
repell_field = fields.RadialField(
    field_data,
    target = (-0.3,0),
    max_radius = 0.6,
    decay_radius = 0.3,
    repelling = True,
)

attract_field = fields.RadialField(
    field_data,
    target = (0.3,0),
    max_radius = 3.0,
    decay_radius = 0.3,
    repelling = False,
)

my_field = fields.VectorField(field_data)
my_field.add(repell_field)
my_field.add(attract_field)

my_plotter.plot(my_field)
