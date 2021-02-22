import test_base  # pylint: disable=import-error

from thundervolt.comm.vision import FiraVision

import json
import logging

def main():
    vision = FiraVision(team_color_yellow=False)

    try:
        while True:
            vision_data = vision.receive_dict()
            print(json.dumps(vision_data, indent=4))

            # Acess test
            print(f"\r\nPosição z da bola: {vision_data['ball']['z']}\r\n")
    except KeyboardInterrupt:
        logging.warn("Ending")


if __name__ == '__main__':
    main()
