from whill_navi2.ros2_launch_utils import *


def generate_launch_description():
    
    f9p_params_yaml_path = get_yaml_path("whill_navi2", "zed_f9p_node_params.yaml")
    f9p_params_data = get_node_params_dict(f9p_params_yaml_path, "zed_f9p_node")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Declare nodes
    zed_f9p_node = Node(
        package="zed_f9p",
        executable="zed_f9p_node",
        parameters=[f9p_params_yaml_path],
        remappings=[
            ("nmea", "nmea"),
            ("nmea_gga", "nmea_gga"),
            ("rtcm", "rtcm")
        ]
    )
    ntrip_node = Node(
        package="zed_f9p",
        executable="ntrip_client_node",
        parameters=[f9p_params_yaml_path],
        remappings=[("rtcm", "rtcm")],
        condition=IfCondition(str(f9p_params_data['use_rtk']))
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
   
    return LaunchDescription([
        zed_f9p_node,
        ntrip_node,
        navsat_node
    ])
