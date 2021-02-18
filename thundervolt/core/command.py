class RobotCommand(object):
    def __init__(self, left_speed=0, right_speed=0):
        self.left_speed = left_speed
        self.right_speed = right_speed

    def __add__(self, other):
        sum_of_left = self.left_speed + other.left_speed
        sum_of_right = self.right_speed + other.right_speed
        return RobotCommand(sum_of_left, sum_of_right)

    def __mul__(self, multiplier):
        return RobotCommand(self.left_speed * multiplier, self.right_speed * multiplier)


class TeamCommand(object):
    def __init__(self):
        self.commands = [RobotCommand() for i in range(3)]

    def reset(self):
        for cmd in self.commands:
            cmd.left_speed = 0
            cmd.right_speed = 0
