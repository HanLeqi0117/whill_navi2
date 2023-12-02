import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    hokuyo_frame_arg = DeclareLaunchArgument(name="hokuyo_frame", default_value="laser_front")   
    velodyne_frame_arg = DeclareLaunchArgument(name="velodyne_frame", default_value="velodyne")
    adis_imu_frame_arg = DeclareLaunchArgument(name="adis_imu_frame", default_value="imu_adis")   
    wit_imu_frame_arg = DeclareLaunchArgument(name="wit_imu_frame", default_value="imu_wit")   
    ublox_frame_arg = DeclareLaunchArgument(name="ublox_frame", default_value="ublox")
    whill_frame_arg = DeclareLaunchArgument(name="whill_frame", default_value="base_link")
     
    tf2_static_hokuyo_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_front_to_base_link',
        arguments = [
            '--x', '0.50', '--y', '0.0', '--z', '0.33', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159', 
            '--frame-id', whill_frame_arg.default_value[0].perform('tf2_static_launch'), 
            '--child-frame-id', hokuyo_frame_arg.default_value[0].perform('tf2_static_launch')
        ]  
    )
    tf2_static_velodyne_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link',
        arguments = [
            '--x', '0.0', '--y', '0.0', '--z', '0.8875', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', whill_frame_arg.default_value[0].perform('tf2_static_launch'), 
            '--child-frame-id', velodyne_frame_arg.default_value[0].perform('tf2_static_launch'), 
        ]  
    )
    tf2_static_adis_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='adis_imu_to_base_link',
        arguments = [
            '--x', '0.44', '--y', '0.23', '--z', '0.2575', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', whill_frame_arg.default_value[0].perform('tf2_static_launch'), 
            '--child-frame-id', adis_imu_frame_arg.default_value[0].perform('tf2_static_launch'), 
        ]
        # IMU TF Arguments
        # arguments = [
        #     '--x', '0.44', '--y', '0.23', '--z', '0.2575', 
        #     '--roll', '-1.57079', '--pitch', '0.0', '--yaw', '0.0', 
        #     '--frame-id', 'base_link', '--child-frame-id', 'imu'
        # ]        
    )
    tf2_static_wit_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wit_imu_to_base_link',
        arguments = [
            '--x', '0.44', '--y', '-0.23', '--z', '0.2575', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5707963', 
            '--frame-id', whill_frame_arg.default_value[0].perform('tf2_static_launch'), 
            '--child-frame-id', wit_imu_frame_arg.default_value[0].perform('tf2_static_launch'), 
        ]       
    )    
    tf2_static_gnss_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_to_base_link',
        arguments = [
            '--x', '0.71', '--y', '0.0', '--z', '0.0175', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', whill_frame_arg.default_value[0].perform('tf2_static_launch'), 
            '--child-frame-id', ublox_frame_arg.default_value[0].perform('tf2_static_launch'), 
        ]
    )
    
    return LaunchDescription([
        hokuyo_frame_arg,
        velodyne_frame_arg,
        adis_imu_frame_arg,
        wit_imu_frame_arg,
        ublox_frame_arg,
        whill_frame_arg,
        tf2_static_hokuyo_node,
        tf2_static_velodyne_node,
        tf2_static_adis_imu_node,
        tf2_static_wit_imu_node,
        tf2_static_gnss_node
    ])