from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('data_harvester'),
        'config',
        'harvester_config.yaml'
    )

    # Creating node for sender with params from config
    data_harvester = Node(
        package='data_harvester',
        executable='data_harvester',
        parameters=[config],
    )

    # Add node to launching
    ld.add_action(data_harvester)
    return ld
