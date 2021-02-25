import base_example

import numpy as np

from thundervolt.core import data
from thundervolt.vector_fields import combinations, plotter

my_plotter = plotter.FieldPlotter('Area')

my_field = combinations.AreaField(
    max_dist = 0.3,
    multiplier = 2.0,
)

my_plotter.plot(my_field)
