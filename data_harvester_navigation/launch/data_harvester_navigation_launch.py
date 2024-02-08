from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Prepare config files
    config_localization = os.path.join(
        get_package_share_directory('data_harvester_navigation'),
        'config',
        'turtlebot4_localization.yaml'
    )

    # Adding launch files from Turtlebot 4 Navigation stack
    turtlebot4_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/localization.launch.py']),
        launch_arguments={
            'params': config_localization,
        }.items()
    )

    ld.add_action(turtlebot4_localization)
    return ld
