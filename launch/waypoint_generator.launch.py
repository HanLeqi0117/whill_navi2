from whill_navi2.ros2_launch_utils import *

def generate_launch_description():

    data_path = DataPath()
    rviz_path = get_rviz_path("whill_navi2", "waypoint_maker.rviz")
    
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
        output="screen"
    )
    ros2bag_play_process = ExecuteProcess(
        cmd=[
                FindExecutable(name="ros2"),
                'bag', 'play', data_path.bag_path
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
        name="amcl",
        namespace='waypoint_generator_launch',
        parameters=[os.path.join(
            get_package_share_directory("whill_navi2"),
            'config', 'params', 'amcl_params.yaml'
        )],
    )
    map_server_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        name="map_server",
        executable="map_server",
        package="waypoint_generator_launch",
        parameters=[{
            "yaml_filename": os.path.join(
                data_path.remap_dir,
                data_path.remap_name + '.yaml'
            )
        }]
    )
    lifecycle_node_group = GroupAction(actions=[
        amcl_lifecycle_node,
        map_server_lifecycle_node
    ])
        
    # Lifecycle Node Configure
    lifecycle_node_configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_maker_rviz2_node,
            on_start=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(amcl_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                ),
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )                
            ]
        )
    )
    # Lifecycle Node Activate
    acml_lifecycle_node_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_lifecycle_node,
            entities=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(amcl_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                TimerAction(
                    period=1.0,
                    actions=[ros2bag_play_process]
                ),
                waypoint_maker_node
            ]
        )
    )  
            
    return LaunchDescription(
        waypoint_maker_rviz2_node,
        lifecycle_node_group
    )

