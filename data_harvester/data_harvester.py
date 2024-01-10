import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from irobot_create_msgs.action import WallFollow
from irobot_create_msgs.msg import DockStatus

from rclpy.qos import qos_profile_sensor_data


class DataHarvester(Node):
    """
    A class that creates clients for all Turtlebot 4 ROS 2 stack services
    """

    def __init__(self):
        super().__init__("data_harvester")

        # Client for wall follow action
        client_callback_group = MutuallyExclusiveCallbackGroup()
        self.wall_follow_client = ActionClient(
            self,
            WallFollow,
            'wall_follow',
            callback_group=client_callback_group,
        )

        # Wait for availability of service
        while not self.wall_follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Wall follow server is not loaded, waiting...')

        # Subscriber to dock status
        self.dock_status = None
        workload_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.subscriber_dock_status_callback,
            qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

        # Timer for starting all workload (indicated callback group that cannot being executed
        # in parallel to avoid deadlocks)
        self.timer_workload = self.create_timer(
            5,
            self.timer_workload_callback,
            callback_group=workload_callback_group,
        )

    def timer_workload_callback(self):

        self.timer_workload.cancel()

        self.get_logger().info("Dock status is: "+ str(self.dock_status))

        # Making request to wall follow
        secs = 0.5 * 60
        self.send_goal_wall_follow(WallFollow.Goal.FOLLOW_RIGHT, secs)

    def subscriber_dock_status_callback(self, msg):
        """
        Callback that update dock status
        :param msg: message with DockStatus type
        :return: None
        """
        self.dock_status = bool(msg.is_docked)

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
        self.destroy_node()

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

    with DataHarvester() as data_harvester:
        try:
            executor.add_node(data_harvester)
            executor.spin()
        except KeyboardInterrupt:
            data_harvester.get_logger().warn("Killing the turtlesim_robonomics_handler...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
