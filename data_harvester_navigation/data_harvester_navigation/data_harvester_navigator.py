import rclpy

from rclpy.executors import MultiThreadedExecutor

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class DataHarvesterNavigator(TurtleBot4Navigator):
    """
    A class for harvesting data with Turtlebot 4 while navigation through the ready map
    """
    def __init__(self):
        super().__init__()
        self.node_name = 'data_harvester_navigator'

        # Set initial pose
        initial_pose = self.getPoseStamped(
            [0.0, 0.0],
            TurtleBot4Directions.SOUTH)
        self.setInitialPose(initial_pose)

        # Undock if docked
        if self.getDockedStatus() is True:
            self.info('Going to undock robot from station')
            self.undock()

        # Wait for Nav2
        self.waitUntilNav2Active()

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

    with DataHarvesterNavigator() as data_harvester_navigator:
        try:
            executor.add_node(data_harvester_navigator)
            executor.spin()
        except KeyboardInterrupt:
            data_harvester_navigator.get_logger().warn("Killing the data harvester navigator...")
            executor.remove_node(data_harvester_navigator)


if __name__ == '__main__':
    main()