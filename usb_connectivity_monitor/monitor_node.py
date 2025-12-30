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