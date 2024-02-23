from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Creating WiFi scanner node
    wifi_scanner_node = Node(
        package='data_harvester_perception',
        executable='wifi_scanner_node',
    )

    # Add node to launching
    ld.add_action(wifi_scanner_node)
    return ld
