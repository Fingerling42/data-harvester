import os
from os.path import dirname
from datetime import datetime
import json
from threading import Event

from zipfile import ZipFile

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from message_filters import ApproximateTimeSynchronizer, Subscriber

from ament_index_python.packages import get_package_share_directory

# Groups that cannot being executed in parallel to avoid deadlocks
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rclpy.executors import MultiThreadedExecutor

from irobot_create_msgs.action import WallFollow, Undock
from irobot_create_msgs.msg import DockStatus, Mouse, IrIntensityVector
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from data_harvester_interfaces.msg import DataHarvesterESPSensors

from slam_toolbox.srv import SaveMap, SerializePoseGraph

from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
import cv2


class DataHarvesterMain(Node):
    """
    A class for handle all necessary function of Turtlebot 4 ROS 2 stack: creating clients, make action requests
    """

    def __init__(self):
        super().__init__("data_harvester_main")  # node name

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

        # SLAM Pose data
        self.robot_position_x = None
        self.robot_position_y = None
        self.robot_position_z = None
        self.robot_orientation_x = None
        self.robot_orientation_y = None
        self.robot_orientation_z = None
        self.robot_orientation_w = None

        # Preparing files for opening
        current_time = datetime.now()
        self.video_name = 'harvesting_process.mp4'
        self.odom_name = 'odom.json'
        self.sensors_name = 'esp_sensors.json'
        workspace_dir = dirname(dirname(dirname(dirname(get_package_share_directory('data_harvester_main')))))
        self.archive_path = (workspace_dir + '/harvested-data-' + current_time.strftime("%d-%m-%Y-%H-%M-%S")
                             + '.zip')
        self.video_path = workspace_dir + '/' + self.video_name
        self.odom_path = workspace_dir + '/' + self.odom_name
        self.sensors_path = workspace_dir + '/' + self.sensors_name
        self.map_path = workspace_dir + '/' + self.map_name.value
        self.odom_file = open(self.odom_path, 'w')
        self.sensors_file = open(self.sensors_path, 'w')

        # Preparing OpenCV for video recording
        self.opencv_bridge = CvBridge()
        self.video_writer = None

        # Callback group to prevent actions executed in parallel
        action_callback_group = MutuallyExclusiveCallbackGroup()
        workload_callback_group = ReentrantCallbackGroup()

        # Creating client for wall follow action and wait for its availability
        self.wall_follow_done_event = Event()
        self.wall_follow_client = ActionClient(
            self,
            WallFollow,
            'wall_follow',
            callback_group=action_callback_group,
        )
        while not self.wall_follow_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Wall follow server is not available, waiting...')

        # Creating client for undock from charging station action and wait for its availability
        self.undock_done_event = Event()
        self.undock_client = ActionClient(
            self,
            Undock,
            'undock',
            callback_group=action_callback_group,
        )
        while not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Undock server is not available, waiting...')

        # Creating clients for map saver service: first for pgm format, second for posegraph format
        map_workload_callback_group = MutuallyExclusiveCallbackGroup()

        self.map_saver_done_event = Event()
        self.map_saver_client = self.create_client(
            SaveMap,
            'slam_toolbox/save_map',
            callback_group=map_workload_callback_group,
        )
        while not self.map_saver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Map saver server is not available, waiting...')

        self.map_serializer_done_event = Event()
        self.map_serializer_client = self.create_client(
            SerializePoseGraph,
            'slam_toolbox/serialize_map',
            callback_group=map_workload_callback_group,
        )

        # Creating subscriber to dock status
        self.dock_status = None
        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.subscriber_dock_status_callback,
            qos_profile_sensor_data,
        )

        # Timer for starting all workload
        self.timer_workload = self.create_timer(
            5,
            self.timer_workload_callback,
            callback_group=workload_callback_group,
        )

        # Creating subscribers to all sensors
        self.subscriber_mouse = Subscriber(
            self,
            Mouse,
            'mouse',
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_imu = Subscriber(
            self,
            Imu,
            'imu',
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_cliff = Subscriber(
            self,
            IrIntensityVector,
            'cliff_intensity',
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_ir_bumper = Subscriber(
            self,
            IrIntensityVector,
            'ir_intensity',
            qos_profile=qos_profile_sensor_data,
        )
        self.subscriber_slam_pose = Subscriber(
            self,
            PoseWithCovarianceStamped,
            'pose',
            qos_profile=qos_profile_sensor_data,
        )

        # Creating a time synchronizer to collect sensors data on the same timestamp
        queue_size = 30
        self.odom_synchronizer = ApproximateTimeSynchronizer(
            [
                self.subscriber_mouse,
                self.subscriber_imu,
                self.subscriber_cliff,
                self.subscriber_ir_bumper,
                self.subscriber_slam_pose,
            ],
            queue_size,
            0.01,
        )
        self.odom_synchronizer.registerCallback(self.record_odom)

        # Creating subscriber to ESP sensors topic
        self.subscriber_esp_sensors = self.create_subscription(
            DataHarvesterESPSensors,
            '/data_harvester/esp_sensors',
            self.subscriber_sensors_callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

        # Creating subscriber to image topic
        self.subscriber_video = self.create_subscription(
            Image,
            'oakd/rgb/preview/image_raw',
            self.subscriber_video_callback,
            qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

    def timer_workload_callback(self):
        """
        Main workload timer callback after which the robot starts
        :return: None
        """
        self.timer_workload.cancel()  # timer is needed only once

        # Check if the robot is docked, if not - start undocking
        if self.dock_status is True:
            self.get_logger().info('The robot is docked, need to undock it first')
            self.send_goal_undock()
            self.undock_done_event.wait()

        # Making request to wall follow
        runtime_sec = int(self.runtime_min.value * 60)
        self.send_goal_wall_follow(WallFollow.Goal.FOLLOW_RIGHT, runtime_sec)
        self.wall_follow_done_event.wait()

        # Save map after finishing wall follow
        self.save_map()
        self.map_saver_done_event.wait()
        self.serialize_map()
        self.map_serializer_done_event.wait()

        self.odom_file.close()
        self.sensors_file.close()
        self.video_writer.release()

        # Create resulting archive with harvested data
        with ZipFile(self.archive_path, 'w') as zip_file:
            self.get_logger().info('Saving zip archive with harvested data to workspace dir...')

            try:
                zip_file.write(self.video_name)
            except FileNotFoundError:
                self.get_logger().error('Video has not been harvested')

            try:
                zip_file.write(self.odom_name)
            except FileNotFoundError:
                self.get_logger().error('Odometry has not been harvested')

            try:
                zip_file.write(self.sensors_name)
            except FileNotFoundError:
                self.get_logger().error('ESP sensors data has not been harvested')

            try:
                zip_file.write(self.map_name.value + '.pgm')
            except FileNotFoundError:
                self.get_logger().error('Map image has not been harvested')

            try:
                zip_file.write(self.map_name.value + '.yaml')
            except FileNotFoundError:
                self.get_logger().error('Map config has not been harvested')

            try:
                zip_file.write(self.map_name.value + '.posegraph')
            except FileNotFoundError:
                self.get_logger().error('Map posegraph has not been harvested')

            try:
                zip_file.write(self.map_name.value + '.data')
            except FileNotFoundError:
                self.get_logger().error('Map database has not been harvested')

        # Garbage removal routine
        try:
            os.remove(self.video_path)
        except FileNotFoundError:
            pass
        try:
            os.remove(self.odom_path)
        except FileNotFoundError:
            pass
        try:
            os.remove(self.sensors_path)
        except FileNotFoundError:
            pass
        try:
            os.remove(self.map_path + '.pgm')
        except FileNotFoundError:
            pass
        try:
            os.remove(self.map_path + '.yaml')
        except FileNotFoundError:
            pass
        try:
            os.remove(self.map_path + '.posegraph')
        except FileNotFoundError:
            pass
        try:
            os.remove(self.map_path + '.data')
        except FileNotFoundError:
            pass

        self.get_logger().info('All done')

    def subscriber_video_callback(self, msg):
        """
        Callback from oakd image topic that starts recording video
        :param msg: Image from topic
        :return: None
        """
        if self.wall_follow_done_event.is_set() is not True:
            self.get_logger().info('Starting video recording...', once=True)
            # Convert Image object to OpenCV.Mat object
            cv_image = self.opencv_bridge.imgmsg_to_cv2(msg)

            # Check if video_writer is needed to be initialized
            if self.video_writer is None:
                self.init_video_writer(msg)

            self.video_writer.write(cv_image)

    def init_video_writer(self, msg):
        """
        A function for initialization of video writer
        :param msg: Image from topic
        :return: None
        """
        size = (msg.width, msg.height)
        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        self.video_writer = cv2.VideoWriter(
            self.video_path,
            fourcc=fourcc,
            fps=30,
            frameSize=size
        )

    def subscriber_sensors_callback(self, esp_sensors_msg):
        """
        A callback function that write all ESP sensors messages to JSON file
        :param esp_sensors_msg: Air quality data from ESP
        :return: None
        """
        if self.wall_follow_done_event.is_set() is not True:
            self.get_logger().info('Starting collecting ESP sensors data...', once=True)
            timestamp = float(esp_sensors_msg.header.stamp.sec + esp_sensors_msg.header.stamp.nanosec * pow(10, -9))

            # ESP sensors
            esp_temperature = float(esp_sensors_msg.temperature)
            esp_humidity = float(esp_sensors_msg.humidity)
            esp_luminosity = int(esp_sensors_msg.luminosity)
            esp_co2 = int(esp_sensors_msg.co2)

            sensors_dict = {
                'timestamp': timestamp,
                'slam_pose': {
                    'robot_position': {
                        'x': self.robot_position_x,
                        'y': self.robot_position_y,
                        'z': self.robot_position_z,
                    },
                    'robot_orientation': {
                        'x': self.robot_orientation_x,
                        'y': self.robot_orientation_y,
                        'z': self.robot_orientation_z,
                        'w': self.robot_orientation_w,
                    },
                },
                'esp_air_sensors': {
                    'temperature': esp_temperature,
                    'humidity': esp_humidity,
                    'luminosity': esp_luminosity,
                    'co2': esp_co2,
                }
            }

            json.dump(sensors_dict, self.sensors_file, indent=4)

    def record_odom(self, mouse_msg, imu_msg, cliff_msg, bumper_ir_msg, pose_msg):
        """
        A callback function that write all odom messages to JSON file
        :param mouse_msg: mouse sensor msg
        :param imu_msg: IMU msg
        :param cliff_msg: IR cliff sensor msg
        :param bumper_ir_msg: IR sensor on bumper msg
        :param pose_msg: Position and orientation from SLAM
        :return: None
        """
        if self.wall_follow_done_event.is_set() is not True:
            self.get_logger().info('Starting collecting odometry...', once=True)
            # Getting all values of sensors readings
            timestamp = float(mouse_msg.header.stamp.sec + mouse_msg.header.stamp.nanosec * pow(10, -9))

            # Pose from SLAM
            self.robot_position_x = float(pose_msg.pose.pose.position.x)
            self.robot_position_y = float(pose_msg.pose.pose.position.y)
            self.robot_position_z = float(pose_msg.pose.pose.position.z)
            self.robot_orientation_x = float(pose_msg.pose.pose.orientation.x)
            self.robot_orientation_y = float(pose_msg.pose.pose.orientation.y)
            self.robot_orientation_z = float(pose_msg.pose.pose.orientation.z)
            self.robot_orientation_w = float(pose_msg.pose.pose.orientation.w)

            # Mouse sensor
            mouse_integrated_x = float(mouse_msg.integrated_x)
            mouse_integrated_y = float(mouse_msg.integrated_y)

            # IMU
            imu_orientation_x = float(imu_msg.orientation.x)
            imu_orientation_y = float(imu_msg.orientation.y)
            imu_orientation_z = float(imu_msg.orientation.z)
            imu_orientation_w = float(imu_msg.orientation.w)
            imu_ang_vel_x = float(imu_msg.angular_velocity.x)
            imu_ang_vel_y = float(imu_msg.angular_velocity.y)
            imu_ang_vel_z = float(imu_msg.angular_velocity.z)
            imu_linear_acc_x = float(imu_msg.linear_acceleration.x)
            imu_linear_acc_y = float(imu_msg.linear_acceleration.y)
            imu_linear_acc_z = float(imu_msg.linear_acceleration.z)

            # Cliff IR sensors
            cliff_side_left_intensity = int(cliff_msg.readings[0].value)
            cliff_front_left_intensity = int(cliff_msg.readings[1].value)
            cliff_front_right_intensity = int(cliff_msg.readings[2].value)
            cliff_side_right_intensity = int(cliff_msg.readings[3].value)

            # IR sensors on front bumper
            bumper_side_left_intensity = int(bumper_ir_msg.readings[0].value)
            bumper_left_intensity = int(bumper_ir_msg.readings[1].value)
            bumper_front_left_intensity = int(bumper_ir_msg.readings[2].value)
            bumper_front_center_left_intensity = int(bumper_ir_msg.readings[3].value)
            bumper_front_center_right_intensity = int(bumper_ir_msg.readings[4].value)
            bumper_front_right_intensity = int(bumper_ir_msg.readings[5].value)
            bumper_right_intensity = int(bumper_ir_msg.readings[6].value)

            # Constructing odom dictionary for JSON dumping
            odom_dict = {'timestamp': timestamp,
                         'slam_pose': {
                             'robot_position': {
                                 'x': self.robot_position_x,
                                 'y': self.robot_position_y,
                                 'z': self.robot_position_z,
                             },
                             'robot_orientation': {
                                 'x': self.robot_orientation_x,
                                 'y': self.robot_orientation_y,
                                 'z': self.robot_orientation_z,
                                 'w': self.robot_orientation_w,
                             },
                         },
                         'mouse_sensor': {
                             'integrated_x': mouse_integrated_x,
                             'integrated_y': mouse_integrated_y,
                         },
                         'imu': {
                             'orientation': {
                                 'x': imu_orientation_x,
                                 'y': imu_orientation_y,
                                 'z': imu_orientation_z,
                                 'w': imu_orientation_w,
                             },
                             'angular_velocity': {
                                 'x': imu_ang_vel_x,
                                 'y': imu_ang_vel_y,
                                 'z': imu_ang_vel_z,
                             },
                             'linear_acceleration': {
                                 'x': imu_linear_acc_x,
                                 'y': imu_linear_acc_y,
                                 'z': imu_linear_acc_z,
                             }
                         },
                         'cliff_ir': {
                             'cliff_side_left': cliff_side_left_intensity,
                             'cliff_front_left': cliff_front_left_intensity,
                             'cliff_front_right': cliff_front_right_intensity,
                             'cliff_side_right': cliff_side_right_intensity,
                         },
                         'bumper_ir': {
                             'bumper_side_left': bumper_side_left_intensity,
                             'bumper_left': bumper_left_intensity,
                             'bumper_front_left': bumper_front_left_intensity,
                             'bumper_front_center_left': bumper_front_center_left_intensity,
                             'bumper_front_center_right': bumper_front_center_right_intensity,
                             'bumper_front_right': bumper_front_right_intensity,
                             'bumper_right': bumper_right_intensity,
                         }
                         }

            json.dump(odom_dict, self.odom_file, indent=4)

    def save_map(self):
        """
        A function for making request to map saver
        :return: None
        """
        # Preparing request
        request = SaveMap.Request()
        request.name.data = self.map_path

        future = self.map_saver_client.call_async(request)
        future.add_done_callback(self.map_saver_result)

    def map_saver_result(self, future):
        """
        A function for processing response from map saver
        :param future:
        :return: None
        """
        response = future.result().result

        if response == 0:
            self.get_logger().info('Map image is successfully saved')
        elif response == 1:
            self.get_logger().error('No map received')
        else:
            self.get_logger().error('Map saving is not completed')

        self.map_saver_done_event.set()

    def serialize_map(self):
        """
        A function for making request to map serializer
        :return: None
        """
        # Preparing request
        request = SerializePoseGraph.Request()
        request.filename = self.map_path

        future = self.map_serializer_client.call_async(request)
        future.add_done_callback(self.map_serializer_result)

    def map_serializer_result(self, future):
        """
        A function for processing response from map serializer
        :param future:
        :return: None
        """
        response = future.result().result
        if response == 0:
            self.get_logger().info('Map is successfully serialized')
        else:
            self.get_logger().error('Map serialization is not completed')

        self.map_serializer_done_event.set()

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
        future
        self.get_logger().info('Finished undocking task')
        self.undock_done_event.set()

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
        self.wall_follow_done_event.set()

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

    with DataHarvesterMain() as data_harvester_main:
        try:
            executor.add_node(data_harvester_main)
            executor.spin()
        except KeyboardInterrupt:
            data_harvester_main.get_logger().warn("Killing the data harvester node...")
            executor.remove_node(data_harvester_main)


if __name__ == '__main__':
    main()
