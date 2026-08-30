from setuptools import setup, find_packages

package_name = 'nidar_geotag'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/geotag.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='singhayush5062@gmail.com',
    description='Detection-to-GPS geotagging and swarm survivor aggregation.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geotag_node = nidar_geotag.geotag_node:main',
            'survivor_aggregator_node = nidar_geotag.survivor_aggregator_node:main',
        ],
    },
)
