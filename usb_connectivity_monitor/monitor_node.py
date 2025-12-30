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

    def is_device_connected(self):
        try:

            result = subprocess.run(['lsusb'], capture_output=True, text=True, check=True)
            output = result.stdout
            

            if not self.vendor_id and not self.product_id:
                return False
            
            target_pattern = r'ID '
            if self.vendor_id:
                target_pattern += re.escape(self.vendor_id)
            else:
                target_pattern += r'[0-9a-fA-F]{4}'
            
            target_pattern += r':'
            
            if self.product_id:
                target_pattern += re.escape(self.product_id)
            else:
                target_pattern += r'[0-9a-fA-F]{4}'

            if re.search(target_pattern, output):
                return True
            else:
                return False
                
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f'Failed to execute lsusb: {e}', throttle_duration_sec=10.0)
            return False
        except FileNotFoundError:
             self.get_logger().error('lsusb command not found. Please install usbutils.', throttle_duration_sec=60.0)
             return False
        except Exception as e:
            self.get_logger().error(f'An error occurred: {e}', throttle_duration_sec=10.0)
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
