import test_base  # pylint: disable=import-error

from thundervolt.comm.control import FiraControl

import time

def main():
    blue_control = FiraControl(team_color_yellow=False)
    yellow_control = FiraControl()

    current_time = time.time()

    while (time.time() - current_time < 5):
        blue_control.transmit(0, -10, 10)
        yellow_control.transmit(0, 10, -10)

    current_time = time.time()

    while (time.time() - current_time < 5):
        blue_control.transmit(1, 10, 10)
        yellow_control.transmit(1, 5, 5)

    current_time = time.time()

    while (time.time() - current_time < 5):
        blue_control.transmit(2, -5, -5)
        yellow_control.transmit(2, -15, -15)

    [blue_control.transmit(i, 0, 0) for i in range(3)]
    [yellow_control.transmit(i, 0, 0) for i in range(3)]


if __name__ == '__main__':
  main()
