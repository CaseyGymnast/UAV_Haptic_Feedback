#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int16

import numpy as np

class Serial_Bridge(Node):
    def __init__(self):
        super.__init__('serial_bridge')

        # Class Variables
        self.pwm = {'Left':0.0, 'Up':0.0, 'Right':0.0, 'Down':0.0}
        self.pwm_as_int = 0000

        # Subscribers
        self.create_subscription(Int16, '/amplitudes', self.callback_amplitudes, 1)
  
    def callback_amplitude(self, msg):
        """Updates the held amplitudes and sends to Teensy"""

        self.pwm_as_int = msg.data
        self.send_serial()

    def send_serial(self):
        data = json.dumps(self.pwm_as_int)

        if self.port.isOpen():
            self.port.write(data.encode('ascii'))
            self.port.flush()
            try:
                incoming = self.port.readline().decode("utf-8")
            except Exception as e:
                print(e)
                pass
        else:
            print("opening error")


def main():
  
    # Define the objects
    serial_bridge = Serial_Bridge()
    rclpy.spin(serial_bridge)

if __name__ == "__main__":
	main()