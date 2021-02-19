import base_example

import numpy as np

from thundervolt.vector_fields import fields, plotter

my_plotter = plotter.FieldPlotter('Radial Field')
repell_field = fields.RadialField(
    target = (-0.3,0),
    max_radius = 0.6,
    decay_radius = 0.1,
    repelling = True,
)

attract_field = fields.RadialField(
    target = (0.3,0),
    max_radius = 3.0,
    decay_radius = 0.6,
    repelling = False,
)

my_field = fields.VectorField()
my_field.add(repell_field)
my_field.add(attract_field)

my_plotter.plot(my_field)
