from .comm.vision import FiraVision
from .comm.control import FiraControl
from .comm.referee import RefereeComm
from .core.data import FieldData
from .core.command import TeamCommand

# Faz toda a parte de comunicação e cria o Coach

class Game():
    def __init__(self, config):
        print("Starting...")

    def run(self):
        while True:
            pass
