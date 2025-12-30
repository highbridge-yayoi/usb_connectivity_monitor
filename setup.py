# SPDX-FileCopyrightText: 2025 highbrige-yayoi <hainu738@gmail.com>
# SPDX-License-Identifier: BSD-3-Clause

from setuptools import setup
import os
from glob import glob

package_name = 'usb_connectivity_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryo takahashi',
    maintainer_email='hainu738@gmail.com',
    description='Monitor specific USB device connectivity and publish status.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = usb_connectivity_monitor.monitor_node:main',
        ],
    },
)