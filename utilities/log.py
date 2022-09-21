import logging
from pathlib import Path

SPOTMICRO = 'SpotMicro'


class Logger:

    def __init__(self):
        logs_folder = 'logs/'
        Path(logs_folder).mkdir(parents=True, exist_ok=True)

        # create file handler which logs even debug messages
        self.logging_file_handler = logging.FileHandler(logs_folder + SPOTMICRO + '.log')
        # self.logging_file_handler.setLevel(logging.INFO)

        # create console handler with a higher log level
        self.logging_stream_handler = logging.StreamHandler()
        # self.logging_stream_handler.setLevel(logging.DEBUG)

        # create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self.logging_file_handler.setFormatter(formatter)
        self.logging_stream_handler.setFormatter(formatter)

    def setup_logger(self, logger_name=None):
        if not logger_name:
            logger_name = SPOTMICRO
        else:
            logger_name = SPOTMICRO + ' ' + logger_name

        logger = logging.getLogger("{:<32}".format(logger_name))

        logger.setLevel(logging.INFO)

        # add the handlers to logger
        logger.addHandler(self.logging_file_handler)
        logger.addHandler(self.logging_stream_handler)

        return logger
