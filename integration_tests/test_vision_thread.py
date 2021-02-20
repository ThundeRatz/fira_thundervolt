import test_base  # pylint: disable=import-error

from thundervolt.comm.vision import FiraVisionThread
from thundervolt.core.data import FieldData

import json
import time

def main():
    test_field_data = FieldData()
    vision = FiraVisionThread(team_color_yellow=False, field_data=test_field_data)

    vision.start()

    while True:
        print(test_field_data)
        time.sleep(1)


if __name__ == '__main__':
    main()
