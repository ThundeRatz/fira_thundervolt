import test_base

from thundervolt.comm.vision import FiraVision

def main():
    vision = FiraVision()

    while True:
        vision_data = vision.receive()
        print(vision_data)

        # Acess test
        print(f"\r\nPosição x da bola: {vision_data.ball.x}\r\n")


if __name__ == '__main__':
  main()
