from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():
    
    data_path = DataPath()
    sensor_launch_path = get_include_launch_path("whill_navi2", "sensor.launch.py")
    kuaro_launch_path = get_include_launch_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_include_launch_path("whill_navi2", "tf2_static.launch.py")
    navigation_launch_path = get_include_launch_path("whill_navi2", "navigation_launch.py")
    nav2_params_yaml_path = get_yaml_path("whill_navi2", "nav2_params.yaml")
    ekf_params_yaml_path = get_yaml_path("whill_navi2", "ekf_node_params.yaml")
    whill_navi2_yaml_path = get_yaml_path("whill_navi2", "whill_navi2_node_params.yaml")
    navsat_ekf_params_yaml_path = get_yaml_path("whill_navi2", "dual_ekf_navsat_params.yaml")
    navigation_rviz_path = get_rviz_path("whill_navi2", "navigation.rviz")
    mapviz_path = get_mapviz_path("whill_navi2", "waypoint_generator_launch.mvc")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()
    mode = LaunchConfiguration("mode")
    # Set the mode with GPS or SLAM
    declare_mode = DeclareLaunchArgument(name="mode", default_value="GPS")
    ld.add_action(declare_mode)
    
    # Include Launch File
    # sensor Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensor_launch_path),
        launch_arguments=[
            ("use_adis_imu", "True"),
            ("use_wit_imu", "True"),
            ("use_velodyne", "True"),
            ("use_ublox", "false"),
            ("use_hokuyo", "True"),
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
    
    # navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments=[
            ("mode", mode),
            ("use_sim_time", "False"),
            ("autostart", "True"),
            ("use_composition", "False"),
            ("use_respawn", "False"),
            ("log_level", "info"),
            ("container_name", "nav2_container"),
            ("namespace", ""),
            ("params_file", nav2_params_yaml_path),
            ("map_path", os.path.join(
                data_path.remap_dir,
                data_path.remap_name + '.yaml'
            ))
        ]
    )
    
    launch_group = GroupAction(
        actions=[
            sensor_launch,
            kuaro_whill_launch,
            tf2_static_launch,
            navigation_launch
        ]
    )
    
    # Parameter YAML file: config/param/ekf_node_params.yaml
    ekf_odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params_yaml_path],
        condition=IfCondition(EqualsSubstitution(mode, "SLAM"))
    )
    # Parameter YAML file: config/param/dual_navsat_params.yaml
    ekf_filter_node_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        parameters=[navsat_ekf_params_yaml_path],
        remappings=[
            ("odometry/filtered", "odometry/filtered/local")
        ]
    )
    ekf_filter_node_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        parameters=[navsat_ekf_params_yaml_path],
        remappings=[
            ("odometry/filtered", "odometry/filtered/global"),
            ("odometry/gps", "odometry/gps"),  
        ]
    )
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        parameters=[navsat_ekf_params_yaml_path],
        remappings=[
            ("gps/fix", "ublox/fix"),
            ("gps/filtered", "gps/filtered"),
            ("imu", "witmotion/imu/data"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/filtered/global")
        ]
    )
    dual_ekf_navsat_group = GroupAction(
        actions=[
            ekf_filter_node_local_node,
            ekf_filter_node_global_node,
            navsat_transform_node
        ],
        condition=IfCondition(EqualsSubstitution(mode, "GPS"))
    )
    whill_navi2_node = Node(
        package="whill_navi2",
        executable="whill_navi2_node",
        name="whill_navi2_node",
        parameters=[
            whill_navi2_yaml_path,
            {
                "read_file": data_path.get_rewapypoint_path()[0],
                'mode': mode
            }
        ]
    )
    navigation_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='navigation_rviz',
        arguments=['-d', navigation_rviz_path],
        output="screen"
    )
    # mapviz_node = Node(
    #     package="mapviz",
    #     executable="mapviz",
    #     parameters=[{"config" : mapviz_path}]
    # )
    waypoint_display_node = Node(
        package="gps_wp_pkg",
        executable="waypoint2marker",
        name="waypoint_display",
        parameters=[{
            "waypoint_read_file" : data_path.get_rewapypoint_path()[0],
            "until_node" : 'navigation_rviz'
        }]
    )
    
    when_rviz_start = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_rviz_node,
            on_start=[
                TimerAction(
                    actions=[
                        launch_group,
                        ekf_odometry_node,
                        dual_ekf_navsat_group,
                        waypoint_display_node
                    ],
                    period=0.2
                ),
                TimerAction(
                    actions=[
                        whill_navi2_node
                    ],
                    period=10.0
                )
            ]
        )
    )
    when_rviz_over = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_rviz_node,
            on_exit=[
                Shutdown(reason='rviz is closed!')
            ]
        )
    )
    
    ld.add_action(navigation_rviz_node)
    ld.add_action(when_rviz_start)
    ld.add_action(when_rviz_over)

    # when_nav2_started = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=nav2_node_group,
    #         on_start=[
    #             # navigation
    #         ]
    #     )
    # )
    
    # ld.add_action(when_nav2_started)

    return ld
    
