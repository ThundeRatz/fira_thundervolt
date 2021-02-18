import base_example

import numpy as np

from thundervolt.core import math, data
from thundervolt.vector_fields import fields, plotter

field_data = data.FieldData()
my_plotter = plotter.FieldPlotter('Tangent Field')
tangent_field = fields.TangentField(
    field_data,
    target = (0,0),
    radius = 0.5,
    max_radius = 1.0,
    decay_radius = 0.3,
)

my_plotter.plot(tangent_field)
