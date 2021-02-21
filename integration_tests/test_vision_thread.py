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
        try:
            print(test_field_data)
            time.sleep(1)
        except KeyboardInterrupt:
            logging.warn("Ending")


if __name__ == '__main__':
    main()
