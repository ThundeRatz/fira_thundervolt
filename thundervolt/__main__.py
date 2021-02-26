import thundervolt
import logging.config
import coloredlogs
import argparse
import json
import os

def main():
    parser = argparse.ArgumentParser(description='ThunderVolt')

    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output (optional)')

    args = parser.parse_args()

    # Config logging
    if args.verbose:
        logging.config.fileConfig('logging.conf')
        log_format_msg = "\r%(asctime)s %(hostname)s %(name)s | %(levelname)s %(message)s"
        coloredlogs.install(level=logging.DEBUG, fmt=log_format_msg)

    # Loads config from file
    parser.add_argument('--config_file', default='config.json')

    args = parser.parse_args()
    config_file = args.config_file

    config_from_file = json.loads(open(config_file, 'r').read())

    # loads env variables config
    env_config = json.loads(open("env_config.json", 'r').read())

    # Sets game config
    game_config = {}

    for key in config_from_file:
        if isinstance(config_from_file[key], dict):
            game_config[key] = {}

            for sub_key in config_from_file[key]:
                game_config[key][sub_key] = os.environ.get(env_config[key][sub_key], config_from_file[key][sub_key])

            continue

        game_config[key] = os.environ.get(env_config[key], config_from_file[key])

    # Create game and run
    my_game = thundervolt.game.Game(game_config)

    try:
        my_game.run()
    except KeyboardInterrupt:
        my_game.end()
        logging.info("Ending the game!")


if __name__ == '__main__':
    main()
