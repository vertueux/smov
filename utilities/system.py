import os
from utilities.log import Logger

log = Logger().setup_logger('System')


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class System(metaclass=Singleton):

    def __init__(self):

        try:
            log.debug('Loading system...')

        except Exception as e:
            log.error('Problem while loading the configuration file', e)

    def temperature(self):
        try:
            temp = os.popen("vcgencmd measure_temp").readline()
            # log.debug("System temperature: " + temp.replace("temp=", "")[:-5])
            return temp.replace("temp=", "")[:-5]
        except:
            return '000'


