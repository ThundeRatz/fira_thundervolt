import base_example

import numpy as np

from thundervolt.core import data
from thundervolt.vector_fields import combinations, plotter

my_plotter = plotter.FieldPlotter('Walls')

my_field = combinations.WallField(
    max_dist = 0.2,
    decay_dist = 0.1,
    multiplier = 1.0,
)

my_plotter.plot(my_field)
