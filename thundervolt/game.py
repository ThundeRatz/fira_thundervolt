import logging

from .comm.vision import FiraVision
from .comm.control import FiraControl
from .comm.referee import RefereeComm
from .core.data import FieldData
from .core.command import TeamCommand
from .coach import Coach


class Game():
    def __init__(self, config):
        vision_ip = config['network']['vision_ip']
        vision_port = int(config['network']['vision_port'])
        referee_ip = config['network']['referee_ip']
        referee_port = int(config['network']['referee_port'])
        control_ip = config['network']['control_ip']
        control_port = int(config['network']['control_port'])
        team_color_yellow = bool(int(config['team_color_yellow']))
        self.use_referee = bool(int(config['use_referee']))

        self.field_data = FieldData()
        self.team_command = TeamCommand()

        self.vision = FiraVision(team_color_yellow, self.field_data, vision_ip, vision_port)
        self.control = FiraControl(team_color_yellow, self.team_command, control_ip, control_port)
        self.referee = RefereeComm(referee_ip, referee_port)

        self.coach = Coach(self.field_data, self.team_command)
        self.coach.setup()

        self.last_state = 'STOP'


    def run(self):
        logging.info("Starting the game!")

        while True:
            # Check if it should respond to referees commands
            if self.use_referee:
                game_state = self.referee.receive().get('foul', 'STOP')
            else:
                game_state = 'GAME_ON'

            # Check if the state has changed, an initialise next state
            if self.last_state != game_state:
                self._state_initialiser(game_state)

            self.vision.update()

            # FSM body
            if game_state == 'GAME_ON':
                self.coach.update()

                self.control.update()
            elif game_state == 'HALT':
                self.control.stop_team()
            else:
                self.control.stop_team()
                self.team_command.reset()


    def _state_initialiser(self, state):
        if state == 'GAME_ON':
            if self.last_state != 'HALT':
                self.coach.initialise()
        else:
            pass

        self.last_state = state


    def end(self):
        self.control.stop_team()

