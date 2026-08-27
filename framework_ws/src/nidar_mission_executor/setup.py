from setuptools import setup, find_packages

package_name = 'nidar_mission_executor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='singhayush5062@gmail.com',
    description='Multi-drone flight orchestration and manual drone control for NIDAR RescueSwarm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor_node = nidar_mission_executor.mission_executor_node:main',
        ],
    },
)
