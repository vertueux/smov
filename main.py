#!/usr/bin/env python3

import sys

from utilities.log import Logger

import multiprocessing

from motion.motion import MotionController
from abort.abort import AbortController
from screen.screen import LCDScreenController
from remote.remote import RemoteControllerController
from events.event import Events

log = Logger().setup_logger()
event = Events()


def process_abort(communication_queues):
    if (event.allow_process_event()):
        abort = AbortController(communication_queues)
        abort.do_process_events_from_queue()


def process_motion(communication_queues):
    if (event.allow_process_event()):
        motion = MotionController(communication_queues)
        motion.do_process_events_from_queues()


def process_remote_controller(communication_queues):
    if (event.allow_process_event()):
        remote = RemoteControllerController(communication_queues)
        remote.do_process_events_from_queues()


# Optional
def process_output_screen(communication_queues):
    if (event.allow_process_event()):
        lcd_screen = LCDScreenController(communication_queues)
        lcd_screen.do_process_events_from_queue()


def create_controllers_queues():
    communication_queues = {'abort': multiprocessing.Queue(10),
                            'motion': multiprocessing.Queue(1),
                            'screen': multiprocessing.Queue(10)}

    log.info('Created the communication queues: ' + ', '.join(communication_queues.keys()))

    return communication_queues


def close_controllers_queues(communication_queues):
    if (event.allow_process_event()):
        log.info('Closing controller queues')

        for queue in communication_queues.items():
            queue.close()
            queue.join_thread()


def main():
    communication_queues = create_controllers_queues()

    # Abort controller
    # Controls the 0E port from PCA9685 to cut the power to the servos conveniently if needed.
    abort = multiprocessing.Process(target=process_abort, args=(communication_queues,))
    abort.daemon = True  # The daemon dies if the parent process dies

    # Start the motion controller
    # Moves the servos
    motion = multiprocessing.Process(target=process_motion, args=(communication_queues,))
    motion.daemon = True

    # Activate Bluetooth controller
    # Let you move the dog using the bluetooth paired device
    remote_controller = multiprocessing.Process(target=process_remote_controller,
                                                           args=(communication_queues,))
    remote_controller.daemon = True

    # Screen
    # Show status of the components in the screen
    screen = multiprocessing.Process(target=process_output_screen,
                                                    args=(communication_queues,))
    screen.daemon = True

    # Start the threads, queues messages are produced and consumed in those
    abort.start()
    motion.start()
    remote_controller.start()
    screen.start()

    if not abort.is_alive():
        log.error("SpotMicro can't work without abort")
        sys.exit(1)

    if not motion.is_alive():
        log.error("SpotMicro can't work without motion")
        sys.exit(1)

    if not remote_controller:
        log.error("SpotMicro can't work without remote_controller")
        sys.exit(1)

    # Make sure the thread/process ends
    abort.join()
    motion.join()
    remote_controller.join()
    screen.join()

    close_controllers_queues(communication_queues)


if __name__ == '__main__':
    log.info('SpotMicro starting...')

    try:
        main()

    except KeyboardInterrupt:
        log.info('Terminated due Control+C was pressed')

    else:
        log.info('Normal termination')
