from .data import Pose2D, RobotData, FieldData


class Receiver:
    def __init__(self):
        self.is_first_frame = True
        self.last_frame = FieldData()

    def receive() -> FieldData:
        # magia e bruxaria pra comunicar com o fira sim

        return FieldData()
