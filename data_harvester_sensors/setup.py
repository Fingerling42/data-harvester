from setuptools import find_packages, setup

package_name = 'data_harvester_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Berman',
    maintainer_email='berman@robonomics.network',
    description='The package for connecting sensor board based on ESP32 and esphome to data harvester',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_sensors_node = data_harvester_sensors.esp32_sensors_node:main',
        ],
    },
)
