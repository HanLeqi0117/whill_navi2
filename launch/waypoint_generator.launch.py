from whill_navi2.ros2_launch_utils import *

def generate_launch_description():

    data_path = DataPath()
    rviz_path = get_rviz_path("whill_navi2", "waypoint_maker.rviz")
    amcl_params_yaml_path = get_yaml_path("whill_navi2", "amcl_params.yaml")
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Node
    waypoint_maker_node = Node(
        package="waypoint_pkg",
        executable="waypoint_maker",
        name='waypoint_maker',
        parameters=[{
            'way_txt_file' : data_path.waypoint_path,
            "point_distance" : 4.0,
            "deg_thresh" : 15.0,
            "deg_chord" : 1.0
        }],
        arguments=['--ros-args', '--log-level', 'info'],
        output="screen"
    )
    ros2bag_play_process = ExecuteProcess(
        cmd=[
                FindExecutable(name="ros2"),
                'bag', 'play', data_path.bag_path, '--rate=0.6'
        ]
    )
    waypoint_maker_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='waypoint_maker_rviz2',
        arguments=['-d', rviz_path]
    )
    
    # Lifecycle Node
    amcl_lifecycle_node = LifecycleNode(
        package="nav2_amcl",
        executable="amcl",
        name='amcl',
        namespace='',
        parameters=[amcl_params_yaml_path],
    )
    map_server_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace='',
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

    # When rviz is launched, launch the lifecycle_manager to start the lifecycle_nodes automatically
    when_rviz_launched = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_maker_rviz2_node,
            on_start=[
                lifecycle_manager_node,
                amcl_lifecycle_node,
                map_server_lifecycle_node
            ]
        )
    )
    # # When amcl is configured, activate itself
    # when_amcl_configured = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=amcl_lifecycle_node,
    #         start_state="configuring",
    #         goal_state="inactive",
    #         entities=[
    #             EmitEvent(
    #                 event=lifecycle.ChangeState(
    #                     lifecycle_node_matcher=matches_action(amcl_lifecycle_node),
    #                     transition_id=Transition.TRANSITION_ACTIVATE
    #                 )
    #             )
    #         ]
    #     )
    # )
    # # When map_server is configured, activate itself
    # when_mapserver_configured = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=map_server_lifecycle_node,
    #         start_state="configuring",
    #         goal_state="inactive",            
    #         entities=[
    #             EmitEvent(
    #                 event=lifecycle.ChangeState(
    #                     lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
    #                     transition_id=Transition.TRANSITION_ACTIVATE
    #                 )
    #             )
    #         ]
    #     )
    # )
    # When waypoint_maker is launched, launch ros2bag in 0.5s
    when_waypoint_maker_launched = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_maker_node,
            on_start=[
                TimerAction(
                    actions=[ros2bag_play_process],
                    period=2.0
                )
            ]
        )
    )
    # When ros2bag is over, shutdown ROSLaunch in 1.0s
    when_ros2bag_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=ros2bag_play_process,
            on_completion=[
                TimerAction(
                    actions=[Shutdown()],
                    period=1.0
                )
            ]
        )
    )
    # Action Group
    action_group = GroupAction(
        actions=[
            when_rviz_launched,
            waypoint_maker_node,
            when_waypoint_maker_launched,
            # when_amcl_configured,
            # when_mapserver_configured,
            when_ros2bag_over
        ]
    )
    
    return LaunchDescription([
        waypoint_maker_rviz2_node,
        # amcl_lifecycle_node,
        # map_server_lifecycle_node,
        action_group        
    ])

