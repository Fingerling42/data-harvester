import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from data_harvester_interfaces.msg import DataHarvesterESPSensors


class ESP32SensorNode(Node):
    """
    A class getting ESP sensors data from serial port and publishing it to corresponding topic
    """

    def __init__(self):
        super().__init__("esp32_sensors_node")  # node name

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
            esp32_sensors_node.get_logger().warn("Killing the ESP32 sensors node...")
            executor.remove_node(esp32_sensors_node)


if __name__ == '__main__':
    main()
