from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():
    
    adis_imu_launch_path = get_include_launch_path("whill_navi2", 'adis16465.launch.py')
    wit_imu_launch_path = get_include_launch_path("whill_navi2", 'wt901.launch.py')
    zed_f9p_launch_path = get_include_launch_path("whill_navi2", 'zed_f9p.launch.py')
    urg_node_launch_path = get_include_launch_path("whill_navi2", 'urg_node2.launch.py')
    web_camera_launch_path = get_include_launch_path("whill_navi2", 'web_camera.launch.py')
    velodyne_launch_path = get_include_launch_path("whill_navi2", 'velodyne-all-nodes-VLP16-launch.py')
    wit_imu_launch_path = get_include_launch_path("whill_navi2", 'zed_f9p.launch.py')
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
        
    # Declare Launch Arguments
    use_adis_imu_arg = DeclareLaunchArgument("use_adis_imu", default_value="false")
    use_wit_imu_arg = DeclareLaunchArgument("use_wit_imu", default_value="false")
    use_velodyne_arg = DeclareLaunchArgument("use_velodyne", default_value="false")
    use_ublox_arg = DeclareLaunchArgument("use_ublox", default_value="false")
    use_hokuyo_arg = DeclareLaunchArgument("use_hokuyo", default_value="false")
    use_web_camera_arg = DeclareLaunchArgument("use_web_camera", default_value="false")
    use_realsense_camera_arg = DeclareLaunchArgument("use_realsense_camera", default_value="false")
    use_zed_camera_arg = DeclareLaunchArgument("use_zed_camera", default_value="false")
                    
    # Include Launch file
    # ADIS IMU Launch
    # Launch Argument YAML file: config/launch_arg/adis_imu_launch_arg.yaml
    adis_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(adis_imu_launch_path),
        condition=IfCondition(LaunchConfiguration(use_adis_imu_arg.name))
    )
    # WIT IMU Launch
    # Parameter YAML file: config/params/wt901_params.yaml
    wit_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wit_imu_launch_path),
        condition=IfCondition(LaunchConfiguration(use_wit_imu_arg.name))
    )
    # Ublox GNSS Launch
    # Launch Argument YAML file: config/launch_arg/ublox_gnss_launch_arg.yaml
    ublox_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_f9p_launch_path),
        condition=IfCondition(LaunchConfiguration(use_ublox_arg.name))        
    )
    # Hokuyo LRF Launch
    # Launch Argument YAML file: config/launch_arg/hokuyo_lrf_launch_arg.yaml
    # Parameter YAML file: config/params/hokuyo_urg_node_params.yaml
    hokuyo_lrf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(urg_node_launch_path),
        launch_arguments=[
            ("auto_start", 'true'),
            ("node_name", 'urg_node2'),
            ("scan_topic_name", 'hokuyo/scan'),
            ("with_rviz", 'false')
        ],
        condition=IfCondition(LaunchConfiguration(use_hokuyo_arg.name))        
    )
    # Web Camera Launch
    # Parameter YAML file: config/params/web_camera_node_params.yaml
    web_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(web_camera_launch_path),
        condition=IfCondition(LaunchConfiguration(use_web_camera_arg.name))        
    )
    # Velodyne Launch
    # Parameter YAML file: config/params/VLP16-velodyne_driver_node-params.yaml
    # Parameter YAML file: config/params/VLP16-velodyne_transform_node-params.yaml
    # Parameter YAML file: config/params/VLP16db.yaml
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velodyne_launch_path),
        launch_arguments=[["with_rviz", "false"]],
        condition=IfCondition(LaunchConfiguration(use_velodyne_arg.name))        
    )
    
    # Group
    launch_group = GroupAction(actions=[
        velodyne_launch,
        web_camera_launch,
        hokuyo_lrf_launch,
        ublox_gnss_launch,
        adis_imu_launch,
        wit_imu_launch
    ])
    
    return LaunchDescription([
        use_adis_imu_arg,
        use_wit_imu_arg,
        use_velodyne_arg,
        use_ublox_arg,
        use_hokuyo_arg,
        use_web_camera_arg,
        use_realsense_camera_arg,
        use_zed_camera_arg,
        launch_group
    ])
