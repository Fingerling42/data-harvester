import rclpy
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from robonomics_ros2_robot_handler.basic_robonomics_handler import BasicRobonomicsHandler


class DataHarvesterRobonomics(BasicRobonomicsHandler):

    def __init__(self) -> None:
        super().__init__()

        # Subscription for chronicler topic with archive file name
        self.subscriber_archive_name = self.create_subscription(
            String,
            'data_harvester/archive_name',
            self.subscriber_archive_name_callback,
            10,
        )
        self.subscriber_archive_name  # prevent unused variable warning

    def subscriber_archive_name_callback(self, msg: String) -> None:
        """
        Method for receiving archive file name msgs from chronicler
        :param msg: msg with turtlesim/msg/String type
        :return: None
        """
        archive_file_name = str(msg.data)
        self.send_datalog_request(archive_file_name, encrypt_recipient_addresses=self.rws_users_list)


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with DataHarvesterRobonomics() as data_harvester_robonomics:
        try:
            executor.add_node(data_harvester_robonomics)
            executor.spin()
        except (KeyboardInterrupt, SystemExit):
            data_harvester_robonomics.get_logger().warn("Killing the Data Harvester Robonomics node...")
            executor.remove_node(data_harvester_robonomics)
            executor.shutdown()


if __name__ == '__main__':
    main()
