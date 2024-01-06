from whill_navi2.ros2_launch_utils import *

def generate_launch_description():
    
    data_path = DataPath()
    sensor_launch_path = get_include_launch_path("whill_navi2", "sensor.launch.py")
    kuaro_launch_path = get_include_launch_path("whill_navi2", "kuaro_whill.launch.py")
    tf2_static_launch_path = get_include_launch_path("whill_navi2", "tf2_static.launch.py")
    navigation_launch_path = get_include_launch_path("whill_navi2", "navigation_launch.py")
    nav2_params_yaml_path = get_yaml_path("whill_navi2", "nav2_params.yaml")
    navigation_rviz_path = get_rviz_path("whill_navi2", "navigation.rviz")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

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
    
    navigation_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='navigation_rviz',
        arguments=['-d', navigation_rviz_path],
        output="screen"
    )
    
    when_rviz_start = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_rviz_node,
            on_start=[
                TimerAction(
                    actions=[launch_group],
                    period=0.5
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
    
    ld = LaunchDescription()
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
    
