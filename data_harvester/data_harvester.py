import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from irobot_create_msgs.action import WallFollow


class DataHarvester(Node):
    """
    A class that creates clients for all Turtlebot 4 ROS 2 stack services
    """

    def __init__(self):
        super().__init__("data_harvester")

        # Client for wall follow action
        self.wall_follow_client = ActionClient(self, WallFollow, 'wall_follow')

    def send_goal_wall_follow(self, follow_side, sec):
        """
        A function for sending goal to wall follow action
        :param follow_side: FOLLOW_RIGHT = -1 or FOLLOW_LEFT = 1
        :param sec: seconds
        :return: None
        """

        # Preparing goal msg
        goal_msg = WallFollow.Goal()
        goal_msg.follow_side = follow_side
        goal_msg.max_runtime.sec = sec
        goal_msg.max_runtime.nanosec = 0

        # Making a request and waiting for feedback
        self.wall_follow_client.wait_for_server()
        send_goal_future = self.wall_follow_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.wall_follow_response_callback)

    def wall_follow_response_callback(self, future):
        """
        A feedback processing
        :param future: feedback response from action
        :return: None
        """

        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Cannot follow wall')
            return
        self.get_logger().info('Wall following goal accepted')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.wall_follow_result_callback)

    def wall_follow_result_callback(self, future):
        """
        A result processing with closing the node
        :param future: result response from action
        :return: None
        """
        self.get_logger().info('The wall follow mission is complete')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    data_harvester = DataHarvester()

    # Making request to wall follow
    secs = 2 * 60
    data_harvester.send_goal_wall_follow(WallFollow.Goal.FOLLOW_RIGHT, secs)

    rclpy.spin(data_harvester)


if __name__ == '__main__':
    main()
