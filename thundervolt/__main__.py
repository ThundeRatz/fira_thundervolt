import thundervolt
import logging

if __name__ == '__main__':

    logging.basicConfig(filename='logfile.log',
                        level=logging.DEBUG)

    my_game = thundervolt.game.Game()

    try:
        my_game.run()
    except KeyboardInterrupt:
        logging.warn("Ending")
