import os
from datetime import datetime
from zipfile import ZipFile
import json
from typing_extensions import Self, Any, Optional

import rclpy

from rclpy.lifecycle import Node, LifecycleState, Publisher, State, TransitionCallbackReturn
from rclpy.node import Client, Subscription

from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import GetParameters

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from irobot_create_msgs.msg import Mouse, IrIntensityVector, DockStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from data_harvester_interfaces.msg import DataHarvesterESPSensors, DataHarvesterWiFiScan


class DataHarvesterChronicler(Node):

    def __init__(self) -> None:
        """
        A class for recording all data that data harvester gets
        """
        super().__init__("data_harvester_chronicler")  # node name

        # Init all variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_pubsub_parameter_client: Optional[Client] = None
        self.ipfs_dir_path: Optional[str] = None
        self.subscriber_mouse: Optional[Subscriber] = None
        self.subscriber_imu: Optional[Subscriber] = None
        self.subscriber_cliff: Optional[Subscriber] = None
        self.subscriber_ir_bumper: Optional[Subscriber] = None
        self.subscriber_esp_sensors: Optional[Subscriber] = None
        self.data_synchronizer: Optional[ApproximateTimeSynchronizer] = None
        self.dock_status: Optional[bool] = None
        self.subscriber_dock_status: Optional[Subscription] = None
        self.subscriber_wifi_scanner: Optional[Subscription] = None
        self.publisher_archive_name: Optional[Publisher] = None
        self.video_path: Optional[str] = None
        self.data_json_path: Optional[str] = None
        self.wifi_json_path: Optional[str] = None
        self.archive_path: Optional[str] = None
        self.data_json_file: Optional[str] = None
        self.wifi_json_file: Optional[str] = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested. This callback is being called when
        the lifecycle node enters the configuring state.
        :return: The state machine either invokes a transition to the inactive state or stays
        in "unconfigured" depending on the return value.
        """

        self.get_logger().info('Configuring chronicler...')

        # Callback groups
        workload_callback_group = ReentrantCallbackGroup()

        # Service for getting IPFS dir from pubsub
        self.get_pubsub_parameter_client = self.create_client(
            GetParameters,
            'robonomics_ros2_pubsub/get_parameters',
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        while not self.get_pubsub_parameter_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pubsub parameter service not available, waiting again...')

        # Make request to get pubsub parameters with IPFS path and RWS user list
        request = GetParameters.Request()
        request.names = ['ipfs_dir_path']
        future = self.get_pubsub_parameter_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        self.ipfs_dir_path = future.result().values[0].string_value

        # Preparing files for opening
        self.data_json_path = os.path.join(self.ipfs_dir_path, 'data.json')
        self.wifi_json_path = os.path.join(self.ipfs_dir_path, 'wifi_list.json')

        current_time = datetime.now()
        self.archive_path = os.path.join(self.ipfs_dir_path,
                                         'harvested-data-' + current_time.strftime("%d-%m-%Y-%H-%M-%S") + '.zip')

        self.data_json_file = open(self.data_json_path, 'w')
        self.data_json_file.write('[\n')

        self.wifi_json_file = open(self.wifi_json_path, 'w')
        self.wifi_json_file.write('[\n')

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
        self.subscriber_esp_sensors = Subscriber(
            self,
            DataHarvesterESPSensors,
            'data_harvester/esp_sensors',
            qos_profile=qos_profile_sensor_data,
        )

        # Creating a time synchronizer to collect sensors data on the same timestamp
        queue_size = 30
        self.data_synchronizer = ApproximateTimeSynchronizer(
            [
                self.subscriber_mouse,
                self.subscriber_imu,
                self.subscriber_cliff,
                self.subscriber_ir_bumper,
                self.subscriber_esp_sensors,
            ],
            queue_size,
            0.1,
        )
        self.data_synchronizer.registerCallback(self.record_data)

        # Creating subscriber to dock status
        self.subscriber_dock_status = self.create_subscription(
            DockStatus,
            'dock_status',
            self.subscriber_dock_status_callback,
            qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

        # Creating subscriber to Wi-Fi scanner
        self.subscriber_wifi_scanner = self.create_subscription(
            DataHarvesterWiFiScan,
            'data_harvester/wifi_networks',
            self.subscriber_wifi_scanner_callback,
            qos_profile_sensor_data,
            callback_group=workload_callback_group,
        )

        # Creating publisher for archive file name after all work is done
        self.publisher_archive_name = self.create_publisher(
            String,
            'data_harvester/archive_name',
            10
        )

        self.get_logger().info('Configuring is successful')
        return TransitionCallbackReturn.SUCCESS

    def subscriber_dock_status_callback(self, msg: DockStatus) -> None:
        """
        Callback that update dock status
        :param msg: Message with DockStatus type
        :return: None
        """
        self.dock_status = bool(msg.is_docked)

    def subscriber_wifi_scanner_callback(self, msg: DataHarvesterWiFiScan) -> None:
        """
        Callback for writing Wi-Fi scanner output to file
        :param msg: DataHarvesterWiFiScan with list of networks
        :return: None
        """
        self.get_logger().info('Starting recording Wi-Fi scan...', once=True)

        # Get timestamp
        timestamp = float(msg.header.stamp.sec + msg.header.stamp.nanosec * pow(10, -9))

        # Getting pose from transform
        try:
            coord_transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())

            robot_position_x = float(coord_transform.transform.translation.x)
            robot_position_y = float(coord_transform.transform.translation.y)
            robot_position_z = float(coord_transform.transform.translation.z)
            robot_orientation_x = float(coord_transform.transform.rotation.x)
            robot_orientation_y = float(coord_transform.transform.rotation.y)
            robot_orientation_z = float(coord_transform.transform.rotation.z)
            robot_orientation_w = float(coord_transform.transform.rotation.w)

        except TransformException:
            self.get_logger().warn('Could not make pose transform')
            robot_position_x = "NaN"
            robot_position_y = "NaN"
            robot_position_z = "NaN"
            robot_orientation_x = "NaN"
            robot_orientation_y = "NaN"
            robot_orientation_z = "NaN"
            robot_orientation_w = "NaN"

        # Fill dict with timestamp and pose
        json_dict = {'timestamp': timestamp,
                     'pose': {
                         'robot_position': {
                             'x': robot_position_x,
                             'y': robot_position_y,
                             'z': robot_position_z,
                         },
                         'robot_orientation': {
                             'x': robot_orientation_x,
                             'y': robot_orientation_y,
                             'z': robot_orientation_z,
                             'w': robot_orientation_w,
                         },
                     },
                     }

        # Add to dict all Wi-Fi SSID and signal straight
        for network in msg.networks:
            bssid = str(network.bssid)
            ssid = str(network.ssid)
            signal = int(network.signal)
            wifi_dict = {
                bssid: {
                    'ssid': ssid,
                    'signal': signal,
                }
            }
            json_dict.update(wifi_dict)

        json_string = json.dumps(json_dict, indent=4)
        self.wifi_json_file.write(json_string + ',\n')

    def record_data(self,
                    mouse_msg: Mouse,
                    imu_msg: Imu,
                    cliff_msg: IrIntensityVector,
                    bumper_ir_msg: IrIntensityVector,
                    esp_sensors_msg: DataHarvesterESPSensors) -> None:
        """
        A callback function that write all odom messages to JSON file
        :param mouse_msg: Mouse sensor msg
        :param imu_msg: IMU msg
        :param cliff_msg: IR cliff sensor msg
        :param bumper_ir_msg: IR sensor on bumper msg
        :param esp_sensors_msg: Air quality data from ESP
        :return: None
        """
        if self.dock_status is False:
            self.get_logger().info('Starting collecting robot data...', once=True)

            # Getting all values of sensors readings
            timestamp = float(mouse_msg.header.stamp.sec + mouse_msg.header.stamp.nanosec * pow(10, -9))

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

            # ESP sensors
            esp_temperature = float(esp_sensors_msg.temperature)
            esp_humidity = float(esp_sensors_msg.humidity)
            esp_luminosity = int(esp_sensors_msg.luminosity)
            esp_co2 = int(esp_sensors_msg.co2)

            # Getting pose from transform
            try:
                coord_transform = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time())

                robot_position_x = float(coord_transform.transform.translation.x)
                robot_position_y = float(coord_transform.transform.translation.y)
                robot_position_z = float(coord_transform.transform.translation.z)
                robot_orientation_x = float(coord_transform.transform.rotation.x)
                robot_orientation_y = float(coord_transform.transform.rotation.y)
                robot_orientation_z = float(coord_transform.transform.rotation.z)
                robot_orientation_w = float(coord_transform.transform.rotation.w)

            except TransformException:
                self.get_logger().warn('Could not make pose transform')
                robot_position_x = "NaN"
                robot_position_y = "NaN"
                robot_position_z = "NaN"
                robot_orientation_x = "NaN"
                robot_orientation_y = "NaN"
                robot_orientation_z = "NaN"
                robot_orientation_w = "NaN"

            # Constructing dictionary for JSON dumping
            json_dict = {'timestamp': timestamp,
                         'pose': {
                             'robot_position': {
                                 'x': robot_position_x,
                                 'y': robot_position_y,
                                 'z': robot_position_z,
                             },
                             'robot_orientation': {
                                 'x': robot_orientation_x,
                                 'y': robot_orientation_y,
                                 'z': robot_orientation_z,
                                 'w': robot_orientation_w,
                             },
                         },
                         'esp_air_sensors': {
                             'temperature': esp_temperature,
                             'humidity': esp_humidity,
                             'luminosity': esp_luminosity,
                             'co2': esp_co2,
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

            json_string = json.dumps(json_dict, indent=4)
            self.data_json_file.write(json_string + ',\n')

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested. The callback is being called when the
        lifecycle node enters the "shutting down" state.
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        """
        # Close files
        self.data_json_file.write(']')
        self.data_json_file.close()

        self.wifi_json_file.write(']')
        self.wifi_json_file.close()

        # Create resulting archive with harvested data
        saved_file_counter = 0
        with ZipFile(self.archive_path, 'w') as zip_file:
            self.get_logger().info('Saving zip archive with harvested data to workspace dir...')

            try:
                zip_file.write(self.data_json_path, os.path.basename(self.data_json_path))
                saved_file_counter += 1
            except FileNotFoundError:
                self.get_logger().error('Robot data has not been harvested')

            try:
                zip_file.write(self.wifi_json_path, os.path.basename(self.wifi_json_path))
                saved_file_counter += 1
            except FileNotFoundError:
                self.get_logger().error('Wi-Fi scanning has not been harvested')

        # Garbage removal routine
        try:
            os.remove(self.data_json_path)
        except FileNotFoundError:
            pass
        try:
            os.remove(self.wifi_json_path)
        except FileNotFoundError:
            pass

        # Publish last message with archive name
        if saved_file_counter > 0:
            archive_name_msg = String()
            archive_name_msg.data = str(os.path.basename(self.archive_path))
            self.publisher_archive_name.publish(archive_name_msg)
            self.publisher_archive_name.wait_for_all_acked()

        # Calling destructors
        self.destroy_lifecycle_publisher(self.publisher_archive_name)
        self.destroy_client(self.get_pubsub_parameter_client)
        self.destroy_subscription(self.subscriber_mouse)
        self.destroy_subscription(self.subscriber_imu)
        self.destroy_subscription(self.subscriber_cliff)
        self.destroy_subscription(self.subscriber_ir_bumper)
        self.destroy_subscription(self.subscriber_esp_sensors)
        self.destroy_subscription(self.subscriber_dock_status)
        self.destroy_subscription(self.subscriber_wifi_scanner)

        self.get_logger().info('All done')
        return TransitionCallbackReturn.SUCCESS

    def __enter__(self) -> Self:
        """
        Enter the object runtime context
        :return: object itself
        """
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        """
        Exit the object runtime context
        :param exc_type: exception that caused the context to be exited
        :param exc_val: exception value
        :param exc_tb: exception traceback
        :return: None
        """


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with DataHarvesterChronicler() as data_harvester_chronicler:
        try:
            executor.add_node(data_harvester_chronicler)
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            data_harvester_chronicler.get_logger().warn("Killing the chronicler node...")
            executor.remove_node(data_harvester_chronicler)
            executor.shutdown()


if __name__ == '__main__':
    main()
