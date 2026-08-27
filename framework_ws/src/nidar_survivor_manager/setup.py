from setuptools import setup, find_packages

package_name = 'nidar_survivor_manager'

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
    description='Runtime Gazebo survivor-dummy spawn/remove for NIDAR RescueSwarm.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'survivor_manager_node = nidar_survivor_manager.survivor_manager_node:main',
        ],
    },
)
