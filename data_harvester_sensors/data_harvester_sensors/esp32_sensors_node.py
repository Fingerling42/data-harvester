import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from data_harvester_interfaces.msg import DataHarvesterESPSensors

import serial
import json


class ESP32SensorNode(Node):
    """
    A class getting ESP sensors data from serial port and publishing it to corresponding topic
    """

    def __init__(self):
        super().__init__("esp32_sensors_node")  # node name
        self.esp_serial = None

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('esp_port', rclpy.Parameter.Type.STRING),
                ('esp_baudrate', rclpy.Parameter.Type.INTEGER)
            ]
        )

        self.esp_port = self.get_parameter('esp_port')
        self.esp_baudrate = self.get_parameter('esp_baudrate')

        # Callback group to not execute reading and publishing in parallel
        sensors_callback_group = MutuallyExclusiveCallbackGroup()

        # Timer for starting reading sensors data
        self.timer_sensors = self.create_timer(
            15,  # 15 sec to configure ESP32
            self.timer_sensors_callback,
            callback_group=sensors_callback_group,
        )

    def timer_sensors_callback(self):
        """
        Timer callback after which sensors readings starts
        :return: None
        """
        self.timer_sensors.cancel()  # timer is needed only once

        # Try to open serial connection to ESP, otherwise shutdown
        try:
            self.esp_serial = serial.Serial()
            self.esp_serial.baudrate = self.esp_baudrate.value
            self.esp_serial.port = self.esp_port.value
            self.esp_serial.open()
            self.get_logger().info('Connected to ESP device on given port')
        except serial.serialutil.SerialException:
            self.get_logger().error('Serial port not found')
            self.executor.remove_node(self)
            rclpy.shutdown()

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

    with ESP32SensorNode() as esp32_sensors_node:
        try:
            executor.add_node(esp32_sensors_node)
            executor.spin()
        except KeyboardInterrupt:
            # Close serial connection and kill node
            esp32_sensors_node.esp_serial.close()
            esp32_sensors_node.get_logger().warn("Killing the ESP32 sensors node...")
            executor.remove_node(esp32_sensors_node)


if __name__ == '__main__':
    main()
