from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config_esp_sensors = os.path.join(
        get_package_share_directory('data_harvester_sensors'),
        'config',
        'esp_config.yaml'
    )

    # Creating ESP32 sensors node with params
    esp32_sensors_node = Node(
        package='data_harvester_sensors',
        executable='esp32_sensors_node',
        parameters=[config_esp_sensors],
    )

    # Add node to launching
    ld.add_action(esp32_sensors_node)
    return ld
