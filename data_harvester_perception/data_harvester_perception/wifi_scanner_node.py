import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from data_harvester_interfaces.msg import DataHarvesterWiFiScan, DataHarvesterWiFiNetwork


class WiFiScannerNode(Node):
    """
    A class getting list of wi-fi networks and publishing it to corresponding topic
    """

    def __init__(self):
        super().__init__("wifi_scanner_node")  # node name

        # Command for running nmcli tool
        self.nmcli_cmd = 'nmcli -e no -g BSSID,SSID,CHAN,RATE,SIGNAL device wifi list --rescan yes'

        scanner_callback_group = MutuallyExclusiveCallbackGroup()

        # Timer for starting scanning
        self.timer_wifi_scanner = self.create_timer(
            5,
            self.timer_wifi_scanner_callback,
            callback_group=scanner_callback_group,
        )

        # Publisher scanned of wi-fi networks
        self.publisher_scanned_networks = self.create_publisher(
            DataHarvesterWiFiScan,
            'data_harvester/wifi_networks',
            qos_profile=qos_profile_sensor_data,
            callback_group=scanner_callback_group,
        )

    def timer_wifi_scanner_callback(self):
        """
        Timer callback for Wi-Fi scanning
        :return: None
        """
        self.get_logger().info("Starting Wi-Fi scanning...", once=True)
        # Run nmcli command and get its output
        nmcli_output = subprocess.run(self.nmcli_cmd,
                                      shell=True,
                                      capture_output=True,
                                      text=True
                                      )

        # Split string to separate lines
        output_lines = nmcli_output.stdout.splitlines()

        # Msg for publishing
        wifi_scanner_msg = DataHarvesterWiFiScan()
        wifi_scanner_msg.networks = []

        # Start parsing output line by line
        for line in output_lines:
            wifi_network = DataHarvesterWiFiNetwork()

            bssid_end = 17
            wifi_network.bssid = str(line[0:bssid_end])

            ssid_end = line.find(":", bssid_end + 1)
            wifi_network.ssid = str(line[bssid_end + 1:ssid_end])

            channel_end = line.find(":", ssid_end + 1)
            wifi_network.channel = int(line[ssid_end + 1:channel_end])

            rate_end = line.find(":", channel_end + 1)
            wifi_network.rate = str(line[channel_end + 1:rate_end])

            signal_end = len(line)
            wifi_network.signal = int(line[rate_end + 1:signal_end])

            wifi_scanner_msg.networks.append(wifi_network)

        # Get timestamp
        wifi_scanner_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_scanned_networks.publish(wifi_scanner_msg)

    def __enter__(self):
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with WiFiScannerNode() as wifi_scanner_node:
        try:
            executor.add_node(wifi_scanner_node)
            executor.spin()
        except KeyboardInterrupt:
            wifi_scanner_node.get_logger().warn("Killing the Wi-Fi scanner node...")
            executor.remove_node(wifi_scanner_node)


if __name__ == '__main__':
    main()
