from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Creating node for sender with params from config
    data_harvester = Node(
        package='data_harvester',
        executable='data_harvester',
    )

    # Add node to launching
    ld.add_action(data_harvester)
    return ld
