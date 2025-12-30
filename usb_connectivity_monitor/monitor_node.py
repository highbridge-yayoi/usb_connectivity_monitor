#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 highbrige-yayoi <hainu738@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import subprocess
import re

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Declare parameters
        self.declare_parameter('vendor_id', '')
        self.declare_parameter('product_id', '')
        self.declare_parameter('check_interval', 1.0)
        
        # Get parameters
        self.vendor_id = str(self.get_parameter('vendor_id').value)
        self.product_id = str(self.get_parameter('product_id').value)
        self.interval = self.get_parameter('check_interval').get_parameter_value().double_value
        
        # Publisher
        self.publisher_ = self.create_publisher(Bool, 'usb_connected', 10)
        
        # Timer
        self.timer = self.create_timer(self.interval, self.check_connection)
        
        self.get_logger().info(f'Monitor Node Started. Target: {self.vendor_id}:{self.product_id}')

    def check_connection(self):
        msg = Bool()
        msg.data = self.is_device_connected()
        self.publisher_.publish(msg)