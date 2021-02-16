class Pose2D:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = 0
        self.y = 0
        self.theta = 0


class RobotData:
    def __init__(self, position: Pose2D = Pose2D(), velocity: Pose2D = Pose2D()):
        self.position = position
        self.velocity = velocity


class FieldData:
    def __init__(self):
        self.robots = [RobotData() for i in range(3)]
        self.foes = [RobotData() for i in range(3)]
        self.ball = RobotData()
