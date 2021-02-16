FIELD_WIDTH = 1.3
FIELD_LENGTH = 1.5

GOAL_WIDTH = 0.4
GOAL_DEPTH = 0.1

class Pose2D:
    def __init__(self, x: float = 0, y: float = 0, theta: float = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f'x: {self.x:.02f}  y: {self.y:.02f}  th: {self.theta:.02f}'

class EntityData:
    def __init__(self, position: Pose2D = Pose2D(), velocity: Pose2D = Pose2D()):
        self.position = position
        self.velocity = velocity

    def __str__(self):
        msg = (
            f'Position: {self.position}\n'
            f'Velocity: {self.velocity}\n'
        )
        return msg

class FieldData:
    def __init__(self):
        self.robots = [EntityData() for i in range(3)]
        self.foes = [EntityData() for i in range(3)]
        self.ball = EntityData()

    def __str__(self):
        msg = f'BALL\n{self.ball}'
        for i in range(3):
            msg += f'\nROBOT_{i}\n{self.robots[i]}'
        for i in range(3):
            msg += f'\nFOE_{i}\n{self.foes[i]}'
        return msg
