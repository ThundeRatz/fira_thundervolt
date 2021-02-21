import base_example

import numpy as np

from thundervolt.core import math
from thundervolt.vector_fields import fields, plotter

my_plotter = plotter.FieldPlotter('Oriented Attracting Field')
my_field = fields.OrientedAttractingField(
    target = (0,0),
    direction = (-1,2),
    nodes_radius = 0.07,
    max_radius = 0.8,
    decay_radius = 0.3,
)

my_plotter.plot(my_field)
