from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():
    
    mkdir_params_yaml_path = get_yaml_path("whill_navi2", "make_dir_node_params.yaml")
    ekf_params_yaml_path = get_yaml_path("whill_navi2", "ekf_node_params.yaml")
    sensor_launch_path = get_include_launch_path("whill_navi2", "sensor.launch.py")
    kuaro_whill_launch_path = get_include_launch_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_include_launch_path("whill_navi2", "tf2_static.launch.py")
    rviz_path = get_rviz_path("whill_navi2", "data_gather_launch.rviz")
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
            ("use_velodyne", "true"),
            ("use_ublox", "false"),
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
    # Parameter YAML file: config/param/ekf_node_params.yaml
    ekf_odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params_yaml_path]
    )
    # Rviz config file: config/rviz2/data_gather_launch.rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='data_gather_rviz2_node',
        arguments=['-d', rviz_path]
    )
    
    # Process
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
    
    # Register Event
    when_make_dir_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=make_dir_node,
            on_exit=[
                launch_group,
                ekf_odometry_node
            ]
        )
    )
    when_launch_complete = RegisterEventHandler(
        OnExecutionComplete(
            target_action=launch_group,
            on_completion=[
                ros2bag_record_process,
                rviz2_node
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
        when_make_dir_complete,
        when_launch_complete,
        when_rviz_exit
    ])
