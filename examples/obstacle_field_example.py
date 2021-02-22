import base_example

import numpy as np

from thundervolt.comm.vision import FiraVision
from thundervolt.core.data import FieldData
from thundervolt.vector_fields import combinations, plotter

def main():
    my_plotter = plotter.FieldPlotter('Obstacles')

    my_field = combinations.ObstaclesField(
        max_radius = 0.1,
        decay_radius = 0.05,
        multiplier = 1.0,
    )

    field_data = FieldData()
    vision = FiraVision(team_color_yellow=False, field_data=field_data)

    vision.update()
    my_field.update(field_data, 2)
    my_plotter.plot(my_field)

if __name__ == '__main__':
    main()

