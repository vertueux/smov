import json
import sys
import os
from utilities.log import Logger
import jmespath  # http://jmespath.org/tutorial.html
import shutil
from pathlib import Path

log = Logger().setup_logger('Configuration')


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Config(metaclass=Singleton):
    abort_GPIO_PORT = 'abort[0].gpio_port'
    screen_I2C_ADDRESS = 'screen[0].lcd_screen[0].address'
    remote_CONTROLLER_DEVICE = 'remote_controller[0].remote[0].device'

    motion_BOARDS_PCA9685_1_ADDRESS = 'motion[*].boards[*].pca9685_1[*].address | [0] | [0] | [0]'
    motion_BOARDS_PCA9685_1_REFERENCE_CLOCK_SPEED = 'motion[*].boards[*].pca9685_1[*].reference_clock_speed | [0] | [0] | [0]'
    motion_BOARDS_PCA9685_1_FREQUENCY = 'motion[*].boards[*].pca9685_1[*].frequency | [0] | [0] | [0]'
    motion_BOARDS_PCA9685_2_ADDRESS = 'motion[*].boards[*].pca9685_2[*].address | [0] | [0] | [0]'
    motion_BOARDS_PCA9685_2_REFERENCE_CLOCK_SPEED = 'motion[*].boards[*].pca9685_2[*].reference_clock_speed | [0] | [0] | [0]'
    motion_BOARDS_PCA9685_2_FREQUENCY = 'motion[*].boards[*].pca9685_2[*].frequency | [0] | [0] | [0]'

    motion_SERVOS_REAR_SHOULDER_LEFT_PCA9685 = 'motion[*].servos[*].rear_shoulder_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_LEFT_CHANNEL = 'motion[*].servos[*].rear_shoulder_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_LEFT_MIN_PULSE = 'motion[*].servos[*].rear_shoulder_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_LEFT_MAX_PULSE = 'motion[*].servos[*].rear_shoulder_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_LEFT_REST_ANGLE = 'motion[*].servos[*].rear_shoulder_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_REAR_LEG_LEFT_PCA9685 = 'motion[*].servos[*].rear_leg_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_LEFT_CHANNEL = 'motion[*].servos[*].rear_leg_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_LEFT_MIN_PULSE = 'motion[*].servos[*].rear_leg_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_LEFT_MAX_PULSE = 'motion[*].servos[*].rear_leg_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_LEFT_REST_ANGLE = 'motion[*].servos[*].rear_leg_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_REAR_FEET_LEFT_PCA9685 = 'motion[*].servos[*].rear_feet_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_LEFT_CHANNEL = 'motion[*].servos[*].rear_feet_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_LEFT_MIN_PULSE = 'motion[*].servos[*].rear_feet_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_LEFT_MAX_PULSE = 'motion[*].servos[*].rear_feet_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_LEFT_REST_ANGLE = 'motion[*].servos[*].rear_feet_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_REAR_SHOULDER_RIGHT_PCA9685 = 'motion[*].servos[*].rear_shoulder_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL = 'motion[*].servos[*].rear_shoulder_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_RIGHT_MIN_PULSE = 'motion[*].servos[*].rear_shoulder_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_RIGHT_MAX_PULSE = 'motion[*].servos[*].rear_shoulder_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_SHOULDER_RIGHT_REST_ANGLE = 'motion[*].servos[*].rear_shoulder_right[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_REAR_LEG_RIGHT_PCA9685 = 'motion[*].servos[*].rear_leg_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_RIGHT_CHANNEL = 'motion[*].servos[*].rear_leg_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_RIGHT_MIN_PULSE = 'motion[*].servos[*].rear_leg_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_RIGHT_MAX_PULSE = 'motion[*].servos[*].rear_leg_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_LEG_RIGHT_REST_ANGLE = 'motion[*].servos[*].rear_leg_right[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_REAR_FEET_RIGHT_PCA9685 = 'motion[*].servos[*].rear_feet_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_RIGHT_CHANNEL = 'motion[*].servos[*].rear_feet_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_RIGHT_MIN_PULSE = 'motion[*].servos[*].rear_feet_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_RIGHT_MAX_PULSE = 'motion[*].servos[*].rear_feet_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_REAR_FEET_RIGHT_REST_ANGLE = 'motion[*].servos[*].rear_feet_right[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_SHOULDER_LEFT_PCA9685 = 'motion[*].servos[*].front_shoulder_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL = 'motion[*].servos[*].front_shoulder_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_LEFT_MIN_PULSE = 'motion[*].servos[*].front_shoulder_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_LEFT_MAX_PULSE = 'motion[*].servos[*].front_shoulder_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_LEFT_REST_ANGLE = 'motion[*].servos[*].front_shoulder_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_LEG_LEFT_PCA9685 = 'motion[*].servos[*].front_leg_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_LEFT_CHANNEL = 'motion[*].servos[*].front_leg_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_LEFT_MIN_PULSE = 'motion[*].servos[*].front_leg_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_LEFT_MAX_PULSE = 'motion[*].servos[*].front_leg_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_LEFT_REST_ANGLE = 'motion[*].servos[*].front_leg_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_FEET_LEFT_PCA9685 = 'motion[*].servos[*].front_feet_left[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_LEFT_CHANNEL = 'motion[*].servos[*].front_feet_left[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_LEFT_MIN_PULSE = 'motion[*].servos[*].front_feet_left[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_LEFT_MAX_PULSE = 'motion[*].servos[*].front_feet_left[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_LEFT_REST_ANGLE = 'motion[*].servos[*].front_feet_left[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685 = 'motion[*].servos[*].front_shoulder_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL = 'motion[*].servos[*].front_shoulder_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_RIGHT_MIN_PULSE = 'motion[*].servos[*].front_shoulder_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_RIGHT_MAX_PULSE = 'motion[*].servos[*].front_shoulder_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_SHOULDER_RIGHT_REST_ANGLE = 'motion[*].servos[*].front_shoulder_right[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_LEG_RIGHT_PCA9685 = 'motion[*].servos[*].front_leg_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_RIGHT_CHANNEL = 'motion[*].servos[*].front_leg_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_RIGHT_MIN_PULSE = 'motion[*].servos[*].front_leg_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_RIGHT_MAX_PULSE = 'motion[*].servos[*].front_leg_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_LEG_RIGHT_REST_ANGLE = 'motion[*].servos[*].front_leg_right[*].rest_angle | [0] | [0] | [0]'

    motion_SERVOS_FRONT_FEET_RIGHT_PCA9685 = 'motion[*].servos[*].front_feet_right[*].pca9685 | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_RIGHT_CHANNEL = 'motion[*].servos[*].front_feet_right[*].channel | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_RIGHT_MIN_PULSE = 'motion[*].servos[*].front_feet_right[*].min_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_RIGHT_MAX_PULSE = 'motion[*].servos[*].front_feet_right[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_FRONT_FEET_RIGHT_REST_ANGLE = 'motion[*].servos[*].front_feet_right[*].rest_angle | [0] | [0] | [0]'

    ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685 = 'motion[*].servos[*].arm_rotation[*].pca9685 | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL = 'motion[*].servos[*].arm_rotation[*].channel | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_ROTATION_MIN_PULSE = 'motion[*].servos[*].arm_rotation[*].min_pulse | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_ROTATION_MAX_PULSE = 'motion[*].servos[*].arm_rotation[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_ARM_ROTATION_REST_ANGLE = 'motion[*].servos[*].arm_rotation[*].rest_angle | [0] | [0] | [0]'

    ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685 = 'motion[*].servos[*].arm_lift[*].pca9685 | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL = 'motion[*].servos[*].arm_lift[*].channel | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_LIFT_MIN_PULSE = 'motion[*].servos[*].arm_lift[*].min_pulse | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_LIFT_MAX_PULSE = 'motion[*].servos[*].arm_lift[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_ARM_LIFT_REST_ANGLE = 'motion[*].servos[*].arm_lift[*].rest_angle | [0] | [0] | [0]'

    ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685 = 'motion[*].servos[*].arm_range[*].pca9685 | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL = 'motion[*].servos[*].arm_range[*].channel | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_RANGE_MIN_PULSE = 'motion[*].servos[*].arm_range[*].min_pulse | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_RANGE_MAX_PULSE = 'motion[*].servos[*].arm_range[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_ARM_RANGE_REST_ANGLE = 'motion[*].servos[*].arm_range[*].rest_angle | [0] | [0] | [0]'

    ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685 = 'motion[*].servos[*].arm_cam_tilt[*].pca9685 | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL = 'motion[*].servos[*].arm_cam_tilt[*].channel | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MIN_PULSE = 'motion[*].servos[*].arm_cam_tilt[*].min_pulse | [0] | [0] | [0]'
    ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_MAX_PULSE = 'motion[*].servos[*].arm_cam_tilt[*].max_pulse | [0] | [0] | [0]'
    motion_SERVOS_ARM_CAM_TILT_REST_ANGLE = 'motion[*].servos[*].arm_cam_tilt[*].rest_angle | [0] | [0] | [0]'

    values = {}

    def __init__(self):

        try:
            log.debug('Loading configuration...')

            self.load_config()
            self.list_modules()

        except Exception as e:
            log.error('Problem while loading the configuration file', e)

    def load_config(self):
        try:
            if not os.path.exists(str(Path.home()) + '/json'):
                shutil.copyfile(str(Path.home()) + '/smov/default',
                                str(Path.home()) + '/json')

            with open(str(Path.home()) + '/json') as json_file:
                self.values = json.load(json_file)
                # log.debug(json.dumps(self.values, indent=4, sort_keys=True))

        except Exception as e:
            log.error("Configuration file don't exist or is not a valid json, aborting.")
            sys.exit(1)

    def list_modules(self):
        log.info('Detected configuration for the modules: ' + ', '.join(self.values.keys()))

    def save_config(self):
        try:
            with open('~/json', 'w') as outfile:
                json.dump(self.values, outfile)
        except Exception as e:
            log.error("Problem saving the configuration file", e)

    def get(self, search_pattern):
        log.debug(search_pattern + ': ' + str(jmespath.search(search_pattern, self.values)))
        return jmespath.search(search_pattern, self.values)

    def get_by_section_name(self, search_pattern):

        PCA9685 = 'motion[*].servos[*].' + str(search_pattern) + '[*].pca9685 | [0] | [0] | [0]'
        CHANNEL = 'motion[*].servos[*].' + str(search_pattern) + '[*].channel | [0] | [0] | [0]'
        MIN_PULSE = 'motion[*].servos[*].' + str(search_pattern) + '[*].min_pulse | [0] | [0] | [0]'
        MAX_PULSE = 'motion[*].servos[*].' + str(search_pattern) + '[*].max_pulse | [0] | [0] | [0]'
        REST_ANGLE = 'motion[*].servos[*].' + str(search_pattern) + '[*].rest_angle | [0] | [0] | [0]'

        PCA9685 = jmespath.search(PCA9685, self.values)

        PCA9685_ADDRESS = 'motion[*].boards[*].pca9685_' + str(PCA9685) + '[*].address | [0] | [0] | [0]'
        PCA9685_REFERENCE_CLOCK_SPEED = 'motion[*].boards[*].pca9685_' + str(PCA9685) + '[*].reference_clock_speed | [0] | [0] | [0]'
        PCA9685_FREQUENCY = 'motion[*].boards[*].pca9685_' + str(PCA9685) + '[*].frequency | [0] | [0] | [0]'

        log.info('PCA9685_ADDRESS: ' + str(jmespath.search(PCA9685_ADDRESS, self.values)))
        log.info('PCA9685_REFERENCE_CLOCK_SPEED: ' + str(jmespath.search(PCA9685_REFERENCE_CLOCK_SPEED, self.values)))
        log.info('PCA9685_FREQUENCY: ' + str(jmespath.search(PCA9685_FREQUENCY, self.values)))
        log.info('CHANNEL: ' + str(jmespath.search(CHANNEL, self.values)))
        log.info('MIN_PULSE: ' + str(jmespath.search(MIN_PULSE, self.values)))
        log.info('MAX_PULSE: ' + str(jmespath.search(MAX_PULSE, self.values)))
        log.info('REST_ANGLE: ' + str(jmespath.search(REST_ANGLE, self.values)))

        return jmespath.search(PCA9685_ADDRESS, self.values), jmespath.search(PCA9685_REFERENCE_CLOCK_SPEED, self.values), jmespath.search(PCA9685_FREQUENCY, self.values), jmespath.search(CHANNEL, self.values), jmespath.search(MIN_PULSE, self.values), jmespath.search(MAX_PULSE, self.values), jmespath.search(REST_ANGLE, self.values)
