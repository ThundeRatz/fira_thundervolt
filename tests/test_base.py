# Import this module to be able to import thundervolt package in the tests

import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))