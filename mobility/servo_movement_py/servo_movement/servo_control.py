#!/usr/bin/python

import rclpy

from ast import arg
from pynput import keyboard
from pynput.keyboard import Listener
from rclpy.node import Node
from i2cpwm_board_msgs.msg import Servo, ServoArray

message = """
Servo Control for 12 Servos. 

You have two options:
---------------------
one: Move one servo manually, all others will be commanded to their center position.
all: Move all servo's manually together.

Commands for controlling the specified servo(s):
------------------------------------------------

q: Quit.
i: Manually increase a servo command value by 1.
d: Manually decrease a servo command value by 1.
m: Set a Servo to maximum.
n: Set a Servo to minimum.
c: Set a Servo to center.
"""

start_commands = ['one', 'all']
key_dict = ['q', 'i', 'd', 'm', 'n', 'c']
number_of_servos = 12

class ServoControl(Node):
    def __init__(self):
        super().__init__('servo_control')
        self.publisher_ = self.create_publisher(ServoArray, "servos_absolute", 10)

class KeyboardInput():
    def __init__(self):
        self._servo_array = ServoArray()
        for i in range(number_of_servos): 
            self._servo_array.servos.append(Servo())

        # Reset all the servos to center.
        self.reset_all_servos_center()

    def reset_all_servos_center(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value = float(306)

    def reset_all_servos_off(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value = float(0)

    def set_all_servos_to_plus_one(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value += float(1)
    
    def set_all_servos_to_minus_one(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value -= float(1)
    
    def set_all_servos_to_center(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value = float(306)
    
    def set_all_servos_to_max(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value = float(520)

    def set_all_servos_to_min(self):
        for i in range(number_of_servos):
            self._servo_array.servos[i].value = float(83)
    
    def execute(self):
        while True:
            print(message)
            usr_input = self.getKey()

            if (usr_input not in start_commands):
                print("Invalid command entered, try again...")
            else:
                if usr_input == 'one':
                    self.reset_all_servos_off()

                    # First get servo number to command.
                    n_srv = -1
                    while (1):
                        print('Which servo to control? Enter a number from 1 to 12: ')
                        usr_input = self.getKey()

                        if usr_input not in range(1,number_of_servos+1):
                            print("Invalid servo number entered, try again")
                        else:
                            n_srv = usr_input - 1
                            break
                
                    print('Enter command, q to go back to option select: ')
                    while (1):
                        usr_input = self.getKey()

                        if usr_input == 'q':
                            break
                        elif usr_input not in key_dict:
                            print('Key not in valid key commands, try again')
                        else:
                            match usr_input:
                                case 'q':
                                    print("Quit...")
                                case 'i':
                                    self._servo_array.servos[n_srv].value += float(1)
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'd':
                                    self._servo_array.servos[n_srv].value -= float(1)
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'm':
                                    self._servo_array.servos[n_srv].value = float(520)
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'm':
                                    self._servo_array.servos[n_srv].value = float(83)
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'c':
                                    self._servo_array.servos[n_srv].value = float(306)
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                elif usr_input == 'all':
                    self.reset_all_servos_center()

                    print('Enter command, q to go back to option select: ')                   
                    while (1):
                        usr_input = self.getKey()

                        if usr_input == 'q':
                            break
                        elif usr_input not in key_dict:
                            print('Key not in valid key commands, try again')
                        else:
                            match usr_input:
                                case 'q':
                                    print("Quit...")
                                case 'i':
                                    self.set_all_servos_to_plus_one()
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'd':
                                    self.set_all_servos_to_minus_one()
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'm':
                                    self.set_all_servos_to_max()
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'm':
                                    self.set_all_servos_to_min()
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))
                                case 'c':
                                    self.set_all_servos_to_center()
                                    print('Servo %2i cmd: %4i'%(n_srv,self._servo_array.servos[n_srv].value))

def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()
    key_input = KeyboardInput()
    key_input.execute()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    servo_control.destroy_node()
    rclpy.shutdonw()

if __name__ == '__main__':
    main()