#!/usr/bin/env python3
import numpy as np
from time import time

import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.msg import Lifter
import gpiod

import os
import ament_index_python.packages

class LifterNode(Node):
    def __init__(self):
        super().__init__('lifter_control_node')

        chip = gpiod.Chip('gpiochip0')
        self.dir = chip.get_line(17)
        self.en = chip.get_line(50)
        self.dir.request(consumer='my_script', type=gpiod.LINE_REQ_DIR_OUT)
        self.en.request(consumer='my_script', type=gpiod.LINE_REQ_DIR_OUT)

        self.create_subscription(Lifter, '/lifter_status', self.lifter_callback, 10)
        self.pub = self.create_publisher()
        self.timer = self.create_timer(0.05, self.timer_callback, 10)
        self.status = 0

    
    def timer_callback(self):
        match self.status:
            case 0:
                self.get_logger.info('Elevator not enabled')
                self.en.set_value(0)
            case 1:
                self.get_logger.info('Elevator enabled, going up')
                self.en.set_value(1)
                self.dir.set_value(0)
            case 2:
                self.get_logger.info('Elevator enabled, going down')
                self.en.set_value(1)
                self.dir.set_value(1)

    
    def lifter_callback(self, msg):
        self.status = msg.status


def main(args=None):
    np.seterr(over='raise')
    rclpy.init(args=args)
    node = LifterNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()