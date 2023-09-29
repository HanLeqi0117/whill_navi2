import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare Launch Arguments
    use_adis_imu_arg = DeclareLaunchArgument("use_adis_imu", "true")
    use_velodyne_arg = DeclareLaunchArgument("use_velodyne", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_ublox", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_rtk", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_hokuyo", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_web_camera", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_realsense_camera", "true")
    use_adis_imu_arg = DeclareLaunchArgument("use_zed_camera", "true")

    # Include Launch file
    if bool(LaunchConfiguration("use_adis_imu").perform(None)):
        adis_imu_launch = IncludeLaunchDescription(
            
        )
    return LaunchDescription([
    ])
