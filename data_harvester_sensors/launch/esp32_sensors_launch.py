from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Creating ESP32 sensors node
    esp32_sensors_node = Node(
        package='data_harvester_sensors',
        executable='esp32_sensors_node',
    )

    # Add node to launching
    ld.add_action(esp32_sensors_node)
    return ld
