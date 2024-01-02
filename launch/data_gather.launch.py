from whill_navi2.ros2_launch_utils import *

def generate_launch_description():
    
    mkdir_params_yaml_path = get_yaml_path("whill_navi2", "make_dir_node_params.yaml")
    ekf_params_yaml_path = get_yaml_path("whill_navi2", "ekf_filter_node.yaml")
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
        name='datagather_rviz2_node',
        arguments=['-d', rviz_path]
    )
    
    # Process
    ros2bag_record_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'bag', 'record', '--all', '-o', data_path.bag_path
        ]
    )

    ## Group
    # Launch
    launch_group = GroupAction(actions=[
        sensor_launch,
        kuaro_whill_launch,
        tf2_static_launch
    ])
    # Node
    node_group = GroupAction(actions=[
        make_dir_node,
        ekf_odometry_node,
        rviz2_node
    ])
    # Action
    action_group = GroupAction(actions=[
        node_group,
        launch_group
    ])

    # Event
    ros2bag_record_event = RegisterEventHandler(
        OnProcessStart(
            target_action=make_dir_node,
            on_start=[
                LogInfo(msg='Sensors are launched, and then start to record the data.'),
                TimerAction(
                    actions=[ros2bag_record_process],
                    period=0.5
                )
            ]
        )
    )

    return LaunchDescription([   
        action_group, 
        ros2bag_record_event
    ])
