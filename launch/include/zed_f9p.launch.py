import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import yaml


def generate_launch_description():

    # Find the Path of params
    params_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        "config",
        "params"
    )
    # Open the yaml files path
    f9p_yaml_params_path = os.path.join(params_path, "zed_f9p_node_params.yaml")
    ntrip_yaml_params_path = os.path.join(params_path, "ntrip_client_node_params.yaml")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Declare launch arguments
    general_debug_arg = DeclareLaunchArgument(name="debug", default_value='true')
    f9p_use_rtk_arg = DeclareLaunchArgument(name="use_rtk", default_value='false')
    f9p_port_arg = DeclareLaunchArgument(name="port", default_value="/dev/ttyACM0")
    ntrip_check_gga_arg = DeclareLaunchArgument(name="check_gga", default_value='false')
    ntrip_mountpoint_arg = DeclareLaunchArgument(name="mountpoint", default_value="32M4NHS")
    frame_id_arg = DeclareLaunchArgument(name="frame_id", default_value="gnss")  
    
    # Declare nodes
    f9p_node = Node(
        package="zed_f9p",
        executable="zed_f9p_node",
        name="zed_f9p_node",
        parameters=[
            f9p_yaml_params_path,
            {
                "debug": LaunchConfiguration("debug"),
                "use_rtk": LaunchConfiguration("use_rtk"),
                "port": LaunchConfiguration("port"),
                "frame_id": LaunchConfiguration("frame_id")
            }
        ]
    )
    ntrip_node = Node(
        package="zed_f9p",
        executable="ntrip_client_node",
        name="ntrip_client_node",
        parameters=[
            ntrip_yaml_params_path,
            {
                "debug": LaunchConfiguration("debug"),
                "check_gga": LaunchConfiguration("check_gga"),
                "mountpoint": LaunchConfiguration("mountpoint")
            }
        ],
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
        node_group,
    ])
