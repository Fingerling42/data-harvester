from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    config_harvester = os.path.join(
        get_package_share_directory('data_harvester'),
        'config',
        'harvester_config.yaml'
    )

    config_slam = os.path.join(
        get_package_share_directory('data_harvester'),
        'config',
        'turtlebot4_slam.yaml'
    )

    # Creating node for sender with params from config
    data_harvester = Node(
        package='data_harvester',
        executable='data_harvester',
        parameters=[config_harvester],
    )

    turtlebot4_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/slam.launch.py']),
        launch_arguments={'params': config_slam}.items()
    )

    # Add node to launching
    ld.add_action(turtlebot4_slam)
    ld.add_action(data_harvester)
    return ld
