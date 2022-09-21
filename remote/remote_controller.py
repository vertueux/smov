import time
import os
import struct
import array
from fcntl import ioctl
import signal
import sys
from utilities.log import Logger
from utilities.config import Config
import utilities.queues as queues

log = Logger().setup_logger('Remote controller')


class RemoteControllerController:

    def __init__(self, communication_queues):

        try:

            log.debug('Starting controller...')

            signal.signal(signal.SIGINT, self.exit_gracefully)
            signal.signal(signal.SIGTERM, self.exit_gracefully)

            # We'll store the states here.
            self.connected_device = False
            self.axis_states = {}
            self.button_states = {}
            self.button_map = []
            self.axis_map = []
            self.jsdev = None
            self.previous_fvalue = 0

            self._abort_queue = communication_queues[queues.abort]
            self._motion_queue = communication_queues[queues.motion]
            self._lcd_screen_queue = communication_queues[queues.screen]

        except Exception as e:
            self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_remote_CONTROLLER_NOK)
            log.error('Remote controller controller initialization problem', e)
            sys.exit(1)

    def exit_gracefully(self, signum, frame):
        log.info('Terminated')
        sys.exit(0)

    def do_process_events_from_queues(self):

        remote_connected_already = False

        while True:

            if self.connected_device and not remote_connected_already:
                self._lcd_screen_queue.put(queues.screen_ACTION_ON)
                self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_remote_CONTROLLER_OK)
                remote_connected_already = True
            else:
                time.sleep(2.5)
                self._abort_queue.put(queues.abort_ACTION_ABORT)
                self._lcd_screen_queue.put(queues.LCD_SCREEN_SHOW_remote_CONTROLLER_SEARCHING)
                remote_connected_already = False
                self.check_for_connected_devices()
                continue

            # Main event loop
            i = 0
            while True:

                try:
                    evbuf = self.jsdev.read(8)
                    if evbuf:
                        buftime, value, type, number = struct.unpack('IhBB', evbuf)

                        if type & 0x80:
                            continue

                        if type & 0x01:
                            button = self.button_map[number]
                            if button:
                                self.button_states[button] = value

                        if type & 0x02:
                            axis = self.axis_map[number]
                            if axis:
                                i += 1
                                fvalue = round(value / 32767.0, 3)

                                #if self.previous_fvalue == fvalue:
                                #    continue

                                self.axis_states[axis] = fvalue
                                self.previous_fvalue = fvalue

                                if axis in ['lx', 'ly', 'lz', 'rx', 'ry', 'rz']:
                                    if i >= 6:
                                        i = 0
                                    else:
                                        continue

                    states = {}
                    states.update(self.button_states)
                    states.update(self.axis_states)

                    # log.debug(states)

                    self._motion_queue.put(states)

                except Exception as e:
                    log.error('Unknown problem while processing the queue of the remote controller controller', e)
                    self._abort_queue.put(queues.abort_ACTION_ABORT)
                    remote_connected_already = False
                    self.check_for_connected_devices()
                    break

    def check_for_connected_devices(self):

        connected_device = Config().get(Config.remote_CONTROLLER_DEVICE)

        log.info('The remote controller is not detected, looking for connected devices')
        self.connected_device = False
        for fn in os.listdir('/dev/input'):
            # if fn.startswith('js'):
            if fn.startswith(str(connected_device)):
                self.connected_device = True

                # These constants were borrowed from linux/input.h
                axis_names = {
                    0x00: 'lx',
                    0x01: 'ly',
                    0x02: 'lz',
                    0x03: 'rx',
                    0x04: 'ry',
                    0x05: 'rz',
                    0x06: 'trottle',
                    0x07: 'rudder',
                    0x08: 'wheel',
                    0x09: 'gas',
                    0x0a: 'brake',
                    0x10: 'hat0x',
                    0x11: 'hat0y',
                    0x12: 'hat1x',
                    0x13: 'hat1y',
                    0x14: 'hat2x',
                    0x15: 'hat2y',
                    0x16: 'hat3x',
                    0x17: 'hat3y',
                    0x18: 'pressure',
                    0x19: 'distance',
                    0x1a: 'tilt_x',
                    0x1b: 'tilt_y',
                    0x1c: 'tool_width',
                    0x20: 'volume',
                    0x28: 'misc',
                }

                button_names = {
                    0x120: 'trigger',
                    0x121: 'thumb',
                    0x122: 'thumb2',
                    0x123: 'top',
                    0x124: 'top2',
                    0x125: 'pinkie',
                    0x126: 'base',
                    0x127: 'base2',
                    0x128: 'base3',
                    0x129: 'base4',
                    0x12a: 'base5',
                    0x12b: 'base6',
                    0x12f: 'dead',
                    0x130: 'a',
                    0x131: 'b',
                    0x132: 'c',
                    0x133: 'x',
                    0x134: 'y',
                    0x135: 'z',
                    0x136: 'tl',
                    0x137: 'tr',
                    0x138: 'tl2',
                    0x139: 'tr2',
                    0x13a: 'select',
                    0x13b: 'start',
                    0x13c: 'mode',
                    0x13d: 'thumbl',
                    0x13e: 'thumbr',

                    0x220: 'dpad_up',
                    0x221: 'dpad_down',
                    0x222: 'dpad_left',
                    0x223: 'dpad_right',

                    # XBox 360 controller uses these codes.
                    0x2c0: 'dpad_left',
                    0x2c1: 'dpad_right',
                    0x2c2: 'dpad_up',
                    0x2c3: 'dpad_down',
                }

                # Open the joystick device.
                fn = '/dev/input/' + str(connected_device)

                log.debug(('Opening %s...' % fn))
                self.jsdev = open(fn, 'rb')

                # Get the device name.
                # buf = bytearray(63)
                buf = array.array('B', [0] * 64)
                ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)  # JSIOCGNAME(len)
                js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
                log.info(('Connected to device: %s' % js_name))

                # Get number of axes and buttons.
                buf = array.array('B', [0])
                ioctl(self.jsdev, 0x80016a11, buf)  # JSIOCGAXES
                num_axes = buf[0]

                buf = array.array('B', [0])
                ioctl(self.jsdev, 0x80016a12, buf)  # JSIOCGBUTTONS
                num_buttons = buf[0]

                # Get the axis map.
                buf = array.array('B', [0] * 0x40)
                ioctl(self.jsdev, 0x80406a32, buf)  # JSIOCGAXMAP

                for axis in buf[:num_axes]:
                    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
                    self.axis_map.append(axis_name)
                    self.axis_states[axis_name] = 0.0

                # Get the button map.
                buf = array.array('H', [0] * 200)
                ioctl(self.jsdev, 0x80406a34, buf)  # JSIOCGBTNMAP

                for btn in buf[:num_buttons]:
                    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
                    self.button_map.append(btn_name)
                    self.button_states[btn_name] = 0

                log.info(('%d axes found: %s' % (num_axes, ', '.join(self.axis_map))))
                log.info(('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map))))

                break
