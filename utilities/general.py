from utilities.log import Logger

log = Logger().setup_logger('System')


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class General(metaclass=Singleton):

    def __init__(self):

        try:
            log.debug('Loading general...')

        except Exception as e:
            log.error('Problem while loading the general singleton', e)

    def maprange(self, a, b, s):
        (a1, a2), (b1, b2) = a, b
        return b1 + ((s - a1) * (b2 - b1) / (a2 - a1))
