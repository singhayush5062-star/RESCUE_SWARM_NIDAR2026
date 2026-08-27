from setuptools import setup, find_packages

package_name = 'nidar_mission_clock'

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
    description='Mission run timing and phase breakdown for NIDAR RescueSwarm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_clock_node = nidar_mission_clock.mission_clock_node:main',
        ],
    },
)
