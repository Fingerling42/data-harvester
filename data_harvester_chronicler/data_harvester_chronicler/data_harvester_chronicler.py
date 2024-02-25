import os
from datetime import datetime
from os.path import dirname
from zipfile import ZipFile
import json

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from irobot_create_msgs.msg import Mouse, IrIntensityVector, DockStatus
from sensor_msgs.msg import Imu, Image
from data_harvester_interfaces.msg import DataHarvesterESPSensors, DataHarvesterWiFiScan

from cv_bridge import CvBridge
import cv2


class DataHarvesterChronicler(Node):
    """
    A class for recording all data that Data Harvester gets
    """

    def __init__(self):
        super().__init__("data_harvester_chronicler")  # node name

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Callback groups
        workload_callback_group = ReentrantCallbackGroup()

        # Preparing files for opening
        current_time = datetime.now()
        self.video_name = 'harvesting_process.mp4'
        self.data_json_name = 'data.json'
        self.wifi_json_name = 'wifi_list.json'
        workspace_dir = dirname(dirname(dirname(dirname(get_package_share_directory('data_harvester_chronicler')))))
        self.archive_path = (workspace_dir + '/harvested-data-' + current_time.strftime("%d-%m-%Y-%H-%M-%S")
                             + '.zip')
        self.video_path = workspace_dir + '/' + self.video_name
        self.data_json_path = workspace_dir + '/' + self.data_json_name
        self.wifi_json_path = workspace_dir + '/' + self.wifi_json_name
        self.data_json_file = open(self.data_json_path, 'w')
        self.wifi_json_file = open(self.wifi_json_path, 'w')

        # # Preparing OpenCV for video recording
        # self.opencv_bridge = CvBridge()
        # video_size = (300, 300)
        # fourcc = cv2.VideoWriter_fourcc(*'avc1')
        # try:
        #     self.video_writer = cv2.VideoWriter(
        #         self.video_path,
        #         fourcc=fourcc,
        #         fps=30,
        #         frameSize=video_size,
        #     )
        # except Exception as e:
        #     self.get_logger().error('Error initializing video writer: %s' % str(e))

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

        # Creating subscriber to image topic
        # self.subscriber_video = self.create_subscription(
        #     Image,
        #     'oakd/rgb/preview/image_raw',
        #     self.subscriber_video_callback,
        #     qos_profile_sensor_data,
        #     callback_group=workload_callback_group,
        # )

        # Creating subscriber to dock status
        self.dock_status = None
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

    # def subscriber_video_callback(self, msg):
    #     """
    #     Callback from oakd image topic that starts recording video
    #     :param msg: Image from topic
    #     :return: None
    #     """
    #     try:
    #         if self.dock_status is False and self.video_writer.isOpened() is True:
    #             self.get_logger().info('Starting video recording...', once=True)
    #             # Convert Image object to OpenCV.Mat object
    #             cv_image = self.opencv_bridge.imgmsg_to_cv2(msg)
    #             self.video_writer.write(cv_image)
    #     except Exception as e:
    #         self.get_logger().error('Error while processing Image: %s' % str(e))

    def subscriber_dock_status_callback(self, msg):
        """
        Callback that update dock status
        :param msg: message with DockStatus type
        :return: None
        """
        self.dock_status = bool(msg.is_docked)

    def subscriber_wifi_scanner_callback(self, msg):
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

        json.dump(json_dict, self.wifi_json_file, indent=4)

    def record_data(self, mouse_msg, imu_msg, cliff_msg, bumper_ir_msg, esp_sensors_msg):
        """
        A callback function that write all odom messages to JSON file
        :param mouse_msg: mouse sensor msg
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

            json.dump(json_dict, self.data_json_file, indent=4)

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
        self.data_json_file.close()
        self.wifi_json_file.close()
        # self.video_writer.release()

        # Create resulting archive with harvested data
        with ZipFile(self.archive_path, 'w') as zip_file:
            self.get_logger().info('Saving zip archive with harvested data to workspace dir...')

            # try:
            #     zip_file.write(self.video_name)
            # except FileNotFoundError:
            #     self.get_logger().error('Video has not been harvested')

            try:
                zip_file.write(self.data_json_name)
            except FileNotFoundError:
                self.get_logger().error('Robot data has not been harvested')

            try:
                zip_file.write(self.wifi_json_name)
            except FileNotFoundError:
                self.get_logger().error('Wi-Fi scanning has not been harvested')

        # Garbage removal routine
        # try:
        #     os.remove(self.video_path)
        # except FileNotFoundError:
        #     pass
        try:
            os.remove(self.data_json_path)
        except FileNotFoundError:
            pass
        try:
            os.remove(self.wifi_json_path)
        except FileNotFoundError:
            pass

        self.get_logger().info('All done')


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    with DataHarvesterChronicler() as data_harvester_chronicler:
        try:
            executor.add_node(data_harvester_chronicler)
            executor.spin()
        except KeyboardInterrupt:
            data_harvester_chronicler.get_logger().warn("Killing the chronicler node...")
            executor.remove_node(data_harvester_chronicler)


if __name__ == '__main__':
    main()
