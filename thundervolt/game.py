import logging
import numpy as np
import random

from .core.utils import vectors_angle
from .comm.vision import FiraVision
from .comm.control import FiraControl
from .comm.referee import RefereeComm
from .comm.replacer import ReplacerComm
from .core.command import TeamCommand
from .core.data import FieldData
from .coach import Coach
from .core import data


class Game():
    def __init__(self, config):
        vision_ip = config['network']['vision_ip']
        vision_port = int(config['network']['vision_port'])
        referee_ip = config['network']['referee_ip']
        referee_port = int(config['network']['referee_port'])
        control_ip = config['network']['control_ip']
        control_port = int(config['network']['control_port'])
        replacer_ip = config['network']['replacer_ip']
        replacer_port = int(config['network']['replacer_port'])
        self.team_color_yellow = bool(int(config['team_color_yellow']))
        self.use_referee = bool(int(config['use_referee']))

        self.field_data = FieldData()
        self.team_command = TeamCommand()

        self.vision = FiraVision(self.team_color_yellow, self.field_data, vision_ip, vision_port)
        self.control = FiraControl(self.team_color_yellow, self.team_command, control_ip, control_port)
        self.referee = RefereeComm(referee_ip, referee_port)
        self.replacer = ReplacerComm(self.team_color_yellow, replacer_ip, replacer_port)

        self.coach = Coach(self.field_data, self.team_command)
        self.coach.setup()

        self.last_state = 'STOP'


    def run(self):
        logging.info("Starting the game!")

        while True:
            # Check if it should respond to referees commands
            if self.use_referee:
                referee_data = self.referee.receive()
                game_state = referee_data.get('foul', 'STOP')
                team_color = referee_data.get('teamcolor')
            else:
                team_color = referee_data.get('NONE')
                game_state = 'GAME_ON'

            self.vision.update()

            # Check if the state has changed, an initialise next state
            if self.last_state != game_state:
                self._state_initialiser(game_state, team_color)

            # FSM body
            if game_state == 'GAME_ON':
                self.coach.update()

                self.control.update()
            elif game_state == 'HALT':
                self.control.stop_team()
            else:
                self.control.stop_team()
                self.team_command.reset()


    def _state_initialiser(self, state, team_color):
        if state == 'GAME_ON':
            if self.last_state != 'HALT':
                self.coach.initialise()
        elif state == 'PENALTY_KICK':
            if not ((team_color == 'YELLOW') ^ (self.team_color_yellow)):
                y_signal = -1 if random.randint(0, 1) == 0 else 1
                x_signal = -1 if team_color == 'YELLOW' else 1

                replacement_list = []

                # Penalty kicker
                kicker_id = 0
                kicker_place = data.EntityData()
                kicker_place.position.x = x_signal * (data.FIELD_LENGTH/4 - data.ROBOT_SIZE * 1.2)
                kicker_place.position.y = y_signal * data.ROBOT_SIZE * 0.3

                ball_entry_point = np.array([
                    x_signal * (data.FIELD_LENGTH/2),
                    -y_signal * (data.GOAL_WIDTH/2 - data.BALL_RADIUS * 2.6)
                ])

                kick_angle = vectors_angle(ball_entry_point - np.array([kicker_place.position.x, kicker_place.position.y])) * 180 / np.pi
                kicker_place.position.theta = kick_angle

                replacement_list.append((kicker_place, kicker_id))

                # Goalkeeper
                goalkeeper_id = 1
                goalkeeper_place = data.EntityData()
                goalkeeper_place.position.x = -x_signal * data.FIELD_LENGTH/2
                goalkeeper_place.position.y = y_signal * 0
                goalkeeper_place.position.theta = 90

                replacement_list.append((goalkeeper_place, goalkeeper_id))

                # Assistent
                assistent_id = 2
                assistent_place = data.EntityData()
                assistent_place.position.x = -x_signal * data.ROBOT_SIZE * 1.5
                assistent_place.position.y = -y_signal * (data.FIELD_WIDTH/2 - 0.125)

                assistent_angle = vectors_angle(ball_entry_point - np.array([assistent_place.position.x, assistent_place.position.y])) * 180 / np.pi
                assistent_place.position.theta = assistent_angle

                replacement_list.append((assistent_place, assistent_id))

                self.replacer.place_team(replacement_list)
        else:
            pass

        self.last_state = state


    def end(self):
        self.control.stop_team()

