import test_base

from thundervolt.comm.referee import RefereeComm

def main():
    referee = RefereeComm()

    while True:
        referee_data = referee.receive()
        print(referee_data)

        # Acess test
        print(f"\r\nEstado do jogo: {referee_data.foul}\r\n")
        print(f"\r\nTempo do jogo: {referee_data.gameHalf}\r\n")
        print(f"\r\nTime: {referee_data.teamcolor}\r\n")


if __name__ == '__main__':
  main()
