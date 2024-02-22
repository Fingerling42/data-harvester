from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    # Prepare config files
    config_localization = os.path.join(
        get_package_share_directory('data_harvester_navigation'),
        'config',
        'turtlebot4_localization.yaml'
    )

    config_navigation = os.path.join(
        get_package_share_directory('data_harvester_navigation'),
        'config',
        'turtlebot4_nav2.yaml'
    )

    # Adding launch files from Turtlebot 4 Navigation stack
    turtlebot4_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/localization.launch.py']),
        launch_arguments={
            'params': config_localization,
            'log_level': 'info',
        }.items()
    )

    turtlebot4_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch'),
            '/nav2.launch.py']),
        launch_arguments={
            'params_file': config_navigation,
        }.items()
    )

    nav2_timer = TimerAction(
        period=20.0,
        actions=[turtlebot4_navigation]
    )

    # Creating node for preparing navigation with data harvesting
    data_harvester_navigator = Node(
        package='data_harvester_navigation',
        executable='data_harvester_navigator',
    )

    navigator_timer = TimerAction(
        period=10.0,
        actions=[data_harvester_navigator]
    )

    ld.add_action(turtlebot4_localization)
    ld.add_action(navigator_timer)
    ld.add_action(nav2_timer)

    return ld
