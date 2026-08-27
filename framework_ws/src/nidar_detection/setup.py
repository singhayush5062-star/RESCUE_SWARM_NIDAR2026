import os
from glob import glob

from setuptools import setup, find_packages

package_name = 'nidar_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='singhayush5062@gmail.com',
    description='Per-drone YOLO person detection for NIDAR RescueSwarm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = nidar_detection.detection_node:main',
            'dataset_capture_node = nidar_detection.dataset_capture_node:main',
        ],
    },
)
