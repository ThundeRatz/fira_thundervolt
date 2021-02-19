import base_example

import numpy as np

from thundervolt.core import math
from thundervolt.vector_fields import fields, plotter

my_plotter = plotter.FieldPlotter('Tangent Field')
tangent_field = fields.TangentField(
    target = (0,0),
    radius = 0.5,
    clockwise = True,
    max_radius = 0.8,
    decay_radius = 0.3,
)

my_plotter.plot(tangent_field)
