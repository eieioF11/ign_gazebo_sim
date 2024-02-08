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
                ('gz_args', ['-r '+os.path.join(pkg_dir,'worlds','tugbot','tugbot_depot.sdf')])]
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
                '/model/tugbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/tugbot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/world/world_demo/model/tugbot/link/camera_front/sensor/color/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                # '/model/tugbot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            remappings=[
                # ('/model/tugbot/tf', '/tf'),
                ('/model/tugbot/cmd_vel', '/cmd_vel'),
                ('/model/tugbot/odometry', '/odom'),
                ('/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan/points', '/tugbot/points'),
                ('/world/world_demo/model/tugbot/link/camera_front/sensor/color/image', '/tugbot/image'),
                ('/world/world_demo/model/tugbot/link/imu_link/sensor/imu/imu', '/tugbot/imu'),
            ],
            output='screen'
        ),
    ]

    return LaunchDescription(list)