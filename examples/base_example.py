# Import this module in the start of your example module to be able to import thundervolt package

import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))
