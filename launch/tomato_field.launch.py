import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory('ign_gazebo_sim')
    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', ['-r '+os.path.join(pkg_dir,'worlds','tomato_field','tomato_field.sdf')])]
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     arguments=[
        #         "--x",
        #         "0.0",
        #         "--y",
        #         "0.0",
        #         "--z",
        #         "0.0",
        #         "--yaw",
        #         "0.0",
        #         "--pitch",
        #         "0.0",
        #         "--roll",
        #         "0.0",
        #         "--frame-id",
        #         "base_link",
        #         "--child-frame-id",
        #         "tugbot/scan_omni/scan_omni",
        #     ],
        # ),
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='bridge_node',
            arguments=[
                '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/front_cliff_laser/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/front_laser/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                # '/model/costar_husky_sensor_config_1/Odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
                '/model/costar_husky_sensor_config_1/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            remappings=[
                ('/model/costar_husky_sensor_config_1/tf', '/tf'),
                ('/model/costar_husky_sensor_config_1/cmd_vel', '/cmd_vel'),
                ('/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/points', '/costar_husky/camera_front/points'),
                ('/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/front_cliff_laser/scan/points', '/costar_husky/front_cliff_laser/points'),
                ('/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/front_laser/scan/points', '/costar_husky/front_laser/points'),
                ('/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/image', '/costar_husky/camera_front/image'),
                ('/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/depth_image', '/costar_husky/camera_front/depth_image'),
            ],
            output='screen'
        ),
    ]

    return LaunchDescription(list)