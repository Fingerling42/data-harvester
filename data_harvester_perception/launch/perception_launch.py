from launch import LaunchDescription

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    esp32_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_harvester_perception'), 'launch'),
            '/esp32_sensors_launch.py'])
    )

    wifi_scanner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('data_harvester_perception'), 'launch'),
            '/wifi_scanner_launch.py'])
    )

    # Add nodes to launching
    ld.add_action(esp32_sensors)
    ld.add_action(wifi_scanner)
    return ld
