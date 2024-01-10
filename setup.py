from setuptools import find_packages, setup

package_name = 'turtlebot4_map_saver'

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
    description='Simple saver of Turtlebot 4 SLAM map with moving along the wall',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot4_map_saver = turtlebot4_map_saver.turtlebot4_map_saver:main',
        ],
    },
)
