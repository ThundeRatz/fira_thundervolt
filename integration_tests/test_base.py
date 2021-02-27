# Import this module in the start of your test module to be able to import thundervolt package

import sys
import pathlib
import logging
import logging.config
import coloredlogs

sys.path.append(str(pathlib.Path(__file__).parent.parent.absolute()))

logging.config.fileConfig('logging.conf')
log_format_msg = "\r%(asctime)s %(hostname)s %(name)s | %(levelname)s %(message)s"
coloredlogs.install(level=logging.DEBUG, fmt=log_format_msg)
