import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml


def generate_launch_description():
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Declare launch arguments
    general_debug_arg = DeclareLaunchArgument(name="debug", default_value='true')
    f9p_use_rtk_arg = DeclareLaunchArgument(name="use_rtk", default_value='false')
    f9p_port_arg = DeclareLaunchArgument(name="port", default_value="/dev/ttyACM0")
    frame_id_arg = DeclareLaunchArgument(name="frame_id", default_value="gnss")  
    serial_baud_arg = DeclareLaunchArgument(name="serial_baud", default_value="230400")  
    rate_arg = DeclareLaunchArgument(name="rate", default_value="20.0")  
    ntrip_usrname_arg = DeclareLaunchArgument(name="ntrip_usrname", default_value="8wna5x52")  
    ntrip_passwd_arg = DeclareLaunchArgument(name="ntrip_passwd", default_value="bh2ket")  
    ntrip_host_arg = DeclareLaunchArgument(name="ntrip_host", default_value="ntrip.ales-corp.co.jp")  
    ntrip_port_arg = DeclareLaunchArgument(name="ntrip_port", default_value="2101")
    ntrip_check_gga_arg = DeclareLaunchArgument(name="check_gga", default_value='false')
    ntrip_mountpoint_arg = DeclareLaunchArgument(name="mountpoint", default_value="32M4NHS")
    
    # Declare nodes
    f9p_node = Node(
        package="zed_f9p",
        executable="zed_f9p_node",
        name="zed_f9p_node",
        parameters=[{
            "debug": LaunchConfiguration("debug"),
            "use_rtk": LaunchConfiguration("use_rtk"),
            "port": LaunchConfiguration("port"),
            "frame_id": LaunchConfiguration("frame_id"),
            "baud": LaunchConfiguration("serial_baud"),
            "rate": LaunchConfiguration("rate")
        }]
    )
    ntrip_node = Node(
        package="zed_f9p",
        executable="ntrip_client_node",
        name="ntrip_client_node",
        parameters=[{
            "debug": LaunchConfiguration("debug"),
            "check_gga": LaunchConfiguration("check_gga"),
            "mountpoint": LaunchConfiguration("mountpoint"),
            "username": LaunchConfiguration("ntrip_usrname"),
            "password": LaunchConfiguration("ntrip_passwd"),
            "host_url": LaunchConfiguration("ntrip_host"),
            "port": LaunchConfiguration("ntrip_port")
        }],
        condition=IfCondition(LaunchConfiguration("use_rtk"))
    )
    navsat_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_topic_driver",
        name="nmea_topic_driver",
        remappings=[
            ("fix", "ublox/fix"),
            ("nmea_sentence", "nmea")
        ]
    )
     
    # Node Group
    node_group = GroupAction(actions=[
        f9p_node,
        ntrip_node,
        navsat_node,        
    ])
        
    return LaunchDescription([
        general_debug_arg,
        f9p_use_rtk_arg,
        f9p_port_arg,
        ntrip_check_gga_arg,
        ntrip_mountpoint_arg,
        frame_id_arg,
        serial_baud_arg,
        rate_arg,
        ntrip_usrname_arg,
        ntrip_passwd_arg,
        ntrip_host_arg,
        ntrip_port_arg,
        node_group,
    ])
