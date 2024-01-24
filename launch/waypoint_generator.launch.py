from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():

    data_path = DataPath()
    rviz_path = get_rviz_path("whill_navi2", "waypoint_maker.rviz")
    mapviz_path = get_mapviz_path("whill_navi2", "waypoint_generator_launch.mvc")
    amcl_params_yaml_path = get_yaml_path("whill_navi2", "amcl_params.yaml")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    ld = LaunchDescription()
    mode = LaunchConfiguration("mode")
    ld.add_action(DeclareLaunchArgument(name="mode", default_value="GPS")) 
    
    waypoint_maker_node = Node(
        package="gps_wp_pkg",
        executable="data2waypoint",
        name='waypoint_maker',
        parameters=[{
            'waypoint_file' : data_path.waypoint_path,
            "waypoint_distance" : 4.0,
            "yaw_deg_thresh" : 5.0,
            "deg_chord" : 0.5,
            "mode" : mode
        }],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("imu", "witmotion/imu/data"),
            ("fix", "gps/filtered")
        ],
        output="screen"
    )
    ros2bag_play_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            'bag', 'play', data_path.bag_path, '--rate=1.0'
        ]
    )
    waypoint_maker_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='waypoint_maker_rviz2',
        arguments=['-d', rviz_path],
    )
    waypoint_maker_mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        parameters=[{"config" : mapviz_path}],
        condition=IfCondition(EqualsSubstitution(mode, "GPS"))
    )
    visualization_nodes = GroupAction(actions=[
        waypoint_maker_mapviz_node,
        waypoint_maker_rviz2_node
    ])
    ld.add_action(visualization_nodes)
    
    # Lifecycle Node
    amcl_lifecycle_node = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[amcl_params_yaml_path],
    )
    map_server_lifecycle_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{
            "yaml_filename": os.path.join(
                data_path.remap_dir,
                data_path.remap_name + '.yaml'
            )
        }]
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{
            'autostart' : True,
            'node_names' : ['amcl', 'map_server']
        }]
    )
    amcl_map_action = GroupAction(
        actions=[
            amcl_lifecycle_node,
            map_server_lifecycle_node,
            lifecycle_manager_node
        ],
        condition=IfCondition(EqualsSubstitution(mode, "SLAM"))
    )
    
    when_rviz_start = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_maker_rviz2_node,
            on_start=[
                waypoint_maker_node,
                amcl_map_action,
                TimerAction(
                    period=2.0,
                    actions=[
                        ros2bag_play_process
                    ]
                )
            ]
        )
    )
    ld.add_action(when_rviz_start)
    when_rosbag_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=ros2bag_play_process,
            on_completion=[
                Shutdown(reason="bag is over")
            ]
        )
    )
    ld.add_action(when_rosbag_over)
    
    return ld

