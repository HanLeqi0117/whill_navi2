from whill_navi2.ros2_launch_utils import *

def generate_launch_description():
    
    data_path = DataPath()
    sensor_launch_path = get_include_launch_path("whill_navi2", "sensor.launch.py")
    kuaro_launch_path = get_include_launch_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_include_launch_path("whill_navi2", "tf2_static.launch.py")
    navigation_rviz_path = get_rviz_path("whill_navi2", "navigation.rviz")
        
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    # Include Launch File
    # sensor Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_path),
        launch_arguments=[
            ("use_adis_imu", "true"),
            ("use_wit_imu", "true"),
            ("use_velodyne", "true"),
            ("use_ublox", "false"),
            ("use_hokuyo", "true"),
            ("use_web_camera", "false"),
            ("use_realsense_camera", "false"),
            ("use_zed_camera", "false")
        ]
    )    
    # kuaro_whill Launch
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kuaro_launch_path)
    )
    # tf2_static Launch
    tf2_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tf2_static_launch_path),
        launch_arguments=[
            ("hokuyo_frame", "laser_front"),
            ("velodyne_frame", "velodyne"),
            ("adis_imu_frame", "imu_adis"),
            ("wit_imu_frame", "imu_wit"),
            ("ublox_frame", "ublox"),
            ("whill_frame", "base_link")
        ]
    )
        
    # Node
    navigation_rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_rviz2_node",
        arguments=["-d", navigation_rviz_path]
    )

    return LaunchDescription(
        
    )
    
