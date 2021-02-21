from .core.data import FieldData
from .core.command import TeamCommand

class Coach(object):
    def __init__(self, field_data: FieldData, team_command: TeamCommand):
        self.field_data = field_data
        self.team_command = team_command

    def update(self):
        # Test update
        self.team_command.commands[0].left_speed += 0.1
        self.team_command.commands[0].right_speed += 0.1

        print(self.team_command)
