import thundervolt
import logging
import argparse
import json
import os

def main():
    # Config logging
    logging.basicConfig(filename='logfile.log',
                        level=logging.DEBUG)

    # Loads config from file
    parser = argparse.ArgumentParser(description='ThunderVolt')
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
