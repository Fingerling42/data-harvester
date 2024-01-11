import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Groups that cannot being executed in parallel to avoid deadlocks
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus

from irobot_create_msgs.action import WallFollow, Undock
from irobot_create_msgs.msg import DockStatus

from slam_toolbox.srv import SaveMap

from rclpy.qos import qos_profile_sensor_data


class DataHarvester(Node):
    """
    A class for handle all necessary function of Turtlebot 4 ROS 2 stack: creating clients, make action requests
    """

    def __init__(self):
        super().__init__("data_harvester") # node name

        # Declare used parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('runtime_min', rclpy.Parameter.Type.DOUBLE),
                ('map_name', rclpy.Parameter.Type.STRING)
            ]
        )

        # Get used parameters
        self.runtime_min = self.get_parameter('runtime_min')
        self.map_name = self.get_parameter('map_name')

        # Creating client for wall follow action and wait for its availability
        client_callback_group = MutuallyExclusiveCallbackGroup()
        self.wall_follow_client = ActionClient(
            self,
            WallFollow,
            'wall_follow',
            callback_group=client_callback_group,
        )
        while not self.wall_follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Wall follow server is not available, waiting...')

        # Creating client for undock from charging station action and wait for its availability
        self.undock_client = ActionClient(
            self,
            Undock,
            'undock',
            callback_group=client_callback_group,
        )
        while not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Undock server is not available, waiting...')

        # Explicitly setting the goal status to be checked next
        self.undock_client_status = GoalStatus.STATUS_EXECUTING

        # Creating client for map saver service
        self.map_saver_client = self.create_client(
            SaveMap,
            'slam_toolbox/save_map',
            callback_group=client_callback_group,
        )
        while not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Map saver server is not available, waiting...')

        # Creating subscriber to dock status
        self.dock_status = None
        workload_callback_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.subscriber_dock_status_callback,
            qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

        # Timer for starting all workload
        self.timer_workload = self.create_timer(
            5,
            self.timer_workload_callback,
            callback_group=workload_callback_group,
        )

    def save_map(self):
        request = SaveMap.Request()
        request.name.data = self.map_name

        future = self.map_saver_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        return future.result()

    def timer_workload_callback(self):
        """
        Main workload timer callback after which the robot starts
        :return: None
        """
        self.timer_workload.cancel() # timer is needed only once

        # Check if the robot is docked, if not - start undocking
        if self.dock_status is True:
            self.get_logger().info("The robot is docked, need to undock it first")
            self.send_goal_undock()
            while self.undock_client_status != GoalStatus.STATUS_SUCCEEDED:
                time.sleep(2)

        # Making request to wall follow
        runtime_sec = int(self.runtime_min.value * 60)
        self.send_goal_wall_follow(WallFollow.Goal.FOLLOW_RIGHT, runtime_sec)

    def subscriber_dock_status_callback(self, msg):
        """
        Callback that update dock status
        :param msg: message with DockStatus type
        :return: None
        """
        self.dock_status = bool(msg.is_docked)

    def send_goal_undock(self):
        """
        A function for sending goal to undock action
        :return: None
        """
        goal_msg = Undock.Goal()
        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Cannot undock, please check')
            return
        self.get_logger().info('Going to undock robot from station')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        self.get_logger().info('Finished undocking task')
        self.undock_client_status = GoalStatus.STATUS_SUCCEEDED

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
        # Checking that goal is accepted
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Cannot start data harvesting, please check')
            return
        self.get_logger().info('Starting data harvesting')

        # Waiting for finishing the goal to proceed for its result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.wall_follow_result_callback)

    def wall_follow_result_callback(self, future):
        runtime = future.result().result.runtime.sec
        self.get_logger().info('The data harvesting is complete, elapsed time: %d sec' % runtime)
        self.get_logger().info('Saving the map to workspace dir...')
        save_map_response = self.save_map()
        if save_map_response.result == 0:
            self.get_logger().info('Map is successfully saved')
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

    with DataHarvester() as data_harvester:
        try:
            executor.add_node(data_harvester)
            executor.spin()
        except KeyboardInterrupt:
            data_harvester.get_logger().warn("Killing the data harvester node...")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_harvester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
