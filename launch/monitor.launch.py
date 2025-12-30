# SPDX-FileCopyrightText: 2025 highbrige-yayoi <hainu738@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vendor_id',
            default_value='',
            description='Target USB Vendor ID (hex string, e.g. 046d)'),
        DeclareLaunchArgument(
            'product_id',
            default_value='',
            description='Target USB Product ID (hex string, e.g. 0825)'),
        DeclareLaunchArgument(
            'check_interval',
            default_value='1.0',
            description='Interval in seconds between checks'),
            
    ])
