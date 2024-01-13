from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():
    
    mkdir_params_yaml_path = get_yaml_path("whill_navi2", "make_dir_node_params.yaml")
    navsat_ekf_params_yaml_path = get_yaml_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    sensor_launch_path = get_include_launch_path("whill_navi2", "sensor.launch.py")
    kuaro_whill_launch_path = get_include_launch_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_include_launch_path("whill_navi2", "tf2_static.launch.py")
    rviz_path = get_rviz_path("whill_navi2", "data_gather_navsat_launch.rviz")
    mapviz_path = get_mapviz_path("whill_navi2", "data_gather_launch.mvc")
    data_path = DataPath()
    data_path.backup_bagfile()
    
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
            ("use_velodyne", "false"),
            ("use_ublox", "true"),
            ("use_hokuyo", "false"),
            ("use_web_camera", "false"),
            ("use_realsense_camera", "false"),
            ("use_zed_camera", "false")
        ]
    )    
    # kuaro_whill Launch
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kuaro_whill_launch_path)
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
    # Parameter YAML file: config/param/make_dir_node_params.yaml
    make_dir_node = Node(
        package='whill_navi2',
        executable='make_dir_node',
        parameters=[mkdir_params_yaml_path]
    )
    # Parameter YAML file: config/param/dual_navsat_params.yaml
    ekf_filter_node_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        parameters=[navsat_ekf_params_yaml_path],
        # Remapping
        # OUTPUT
        # From "odometry/filtered" to NAVSAT
        remappings=[
            ("odometry/filtered", "odometry/filtered/local")
        ]
    )
    ekf_filter_node_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        parameters=[navsat_ekf_params_yaml_path],
        # Remapping
        # INPUT
        # From "odometry/gps" to NAVSAT
        # OUTPUT
        # From "odometry/filtered" to NAVSAT
        remappings=[
            ("odometry/filtered", "odometry/filtered/global"),
            ("odometry/gps", "odometry/gps"),  
        ]
    )
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        parameters=[navsat_ekf_params_yaml_path],
        # Remapping
        # INPUT
        # From "gps/fix" to UBLOX
        # From "imu" to WITMOTION
        # From "odometry/filtered" to GLOBAL_EKF
        # OUTPUT
        # From "odometry/gps" to GLOBAL_EKF
        # From "gps/filtered" to WHICH_NEEDS_IT
        remappings=[
            ("gps/fix", "ublox/fix"),
            ("gps/filtered", "gps/filtered"),
            ("imu", "witmotion/imu/data"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/filtered/global")
        ]
    )
    # Rviz config file: config/rviz2/data_gather_launch.rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='datagather_rviz2_node',
        arguments=['-d', rviz_path]
    )
    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        parameters=[{"config" : mapviz_path}]
    )
    initialize_origin_node = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin_node",
        remappings=[
            ("fix", "ublox/fix")
        ]
    )
        
    ros2bag_record_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'bag', 'record', '--all', '-o', data_path.bag_path
        ]
    )

    launch_group = GroupAction(actions=[
        sensor_launch,
        kuaro_whill_launch,
        tf2_static_launch
    ])
    dual_ekf_navsat_nodes = GroupAction(actions=[
        ekf_filter_node_local_node,
        ekf_filter_node_global_node,
        navsat_transform_node,
    ])
    visualization_tools = GroupAction(actions=[
        rviz2_node,
        mapviz_node,
        initialize_origin_node
    ])
    
    # Register Event
    when_make_dir_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=make_dir_node,
            on_exit=[
                launch_group,
                dual_ekf_navsat_nodes
            ]
        )
    )
    when_launch_complete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=launch_group,
            on_completion=[
                ros2bag_record_process,
                visualization_tools
            ]
        )
    )
    when_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz2_node,
            on_exit=[
                Shutdown()
            ]
        )
    )

    return LaunchDescription([   
        make_dir_node,
        when_make_dir_exit,
        when_launch_complete,
        when_rviz_exit
    ])
