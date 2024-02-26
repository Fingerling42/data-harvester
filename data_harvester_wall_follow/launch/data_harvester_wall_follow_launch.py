from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    config_harvester = os.path.join(
        get_package_share_directory('data_harvester_wall_follow'),
        'config',
        'harvester_config.yaml'
    )

    config_slam = os.path.join(
        get_package_share_directory('data_harvester_wall_follow'),
        'config',
        'turtlebot4_slam.yaml'
    )

    # Creating node for wall follow with params from config
    data_harvester_wall_follow = Node(
        package='data_harvester_wall_follow',
        executable='data_harvester_wall_follow',
        parameters=[config_harvester],
    )

    # Adding launch file from Turtlebot 4 SLAM
    turtlebot4_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/slam.launch.py']),
        launch_arguments={
            'params': config_slam,
            'sync': 'false',
        }.items()
    )

    # Add nodes to launching
    ld.add_action(turtlebot4_slam)
    ld.add_action(data_harvester_wall_follow)
    return ld
