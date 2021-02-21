import thundervolt
import logging
import argparse
import json

def main():
    parser = argparse.ArgumentParser(description='ThunderVolt')
    parser.add_argument('--config_file', default='config.json')

    args = parser.parse_args()
    config_file = args.config_file

    config = json.loads(open(config_file, 'r').read())

    print(config)

    logging.basicConfig(filename='logfile.log',
                        level=logging.DEBUG)

    my_game = thundervolt.game.Game(config)

    try:
        my_game.run()
    except KeyboardInterrupt:
        logging.warn("Ending")


if __name__ == '__main__':
    main()
