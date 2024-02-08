import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import launch


def generate_launch_description():
    pkg_dir = get_package_share_directory('ign_gazebo_sim')
    launch_list = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.2",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "imu",
                "--child-frame-id",
                "base_link",
            ],
        ),
        Node(
            package="glim_ros",
            executable="glim_rosnode",
            output="screen",
            parameters=[
                {"config_path": os.path.join(pkg_dir, "config", "glim")},
                {"debug" : False}
            ]
        ),
    ]

    return LaunchDescription(launch_list)
