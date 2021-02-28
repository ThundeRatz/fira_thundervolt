import test_base  # pylint: disable=import-error

from thundervolt.comm.vision import FiraVisionThread
from thundervolt.core.data import FieldData

import json
import time
import logging

def main():
    test_field_data = FieldData()
    vision = FiraVisionThread(team_color_yellow=False, field_data=test_field_data)

    vision.start()

    try:
        while True:
            print(test_field_data)
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Ending")


if __name__ == '__main__':
    main()
