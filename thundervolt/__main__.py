import thundervolt

if __name__ == '__main__':
    my_game = thundervolt.game.Game()

    try:
        my_game.run()
    except KeyboardInterrupt:
        print("\nEnding")
