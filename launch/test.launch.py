from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():

    data_path = DataPath()
    rviz_path = get_rviz_path("whill_navi2", "waypoint_editor.rviz")
    # mapviz_path = get_mapviz_path("whill_navi2", "")
    read_path, write_path = data_path.get_rewapypoint_path()
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    ld = LaunchDescription()
    mode = LaunchConfiguration("mode")
    declare_mode = DeclareLaunchArgument(name="mode", default_value="SLAM", description="SLAM or GPS")
    ld.add_action(declare_mode)
    
    # Node
    waypoint_editor_rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="waypoint_editor_rviz2",
        arguments=["-d", rviz_path],
        output="screen"
    )
    ld.add_action(waypoint_editor_rviz2_node)
    
    waypoint_editor_node = Node(
        package="gps_wp_pkg",
        executable="waypoint_editor",
        name="waypoint_editor",
        parameters=[{
            "read_file_name": read_path,
            "write_file_name": write_path  
        }],
        output="screen"
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{
            'autostart' : True,
            'node_names' : ['map_server']
        }]
    )
    # Lifecycle Node
    map_server_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        parameters=[{
            "yaml_filename": os.path.join(
                data_path.remap_dir,
                data_path.remap_name + '.yaml'
            )
        }],
        output="screen"
    )
    map_action = GroupAction(
        actions=[
            map_server_lifecycle_node,
            lifecycle_manager_node
        ],
        condition=IfCondition(EqualsSubstitution(mode, "SLAM"))
    )
    
    when_rviz_start = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_editor_rviz2_node,
            on_start=[
                waypoint_editor_node,
                map_action
            ]
        )
    )
    ld.add_action(when_rviz_start)
    when_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_editor_rviz2_node,
            on_exit=[
                Shutdown()
            ]
        )
    )
    ld.add_action(when_rviz_exit)
    
    return ld
