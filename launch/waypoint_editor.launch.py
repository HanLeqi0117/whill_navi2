import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, Shutdown, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import Node, LifecycleNode
from lifecycle_msgs.msg import Transition

# input: path of directory.
# output: list of the flie names in the directory
def list_files_in_directory(path):
    file_list = []
    for root, dirs, files in os.walk(path):
        for file in files:
            file_list.append(file)
    return sorted(file_list)

def generate_launch_description():
    
    with open(os.path.join(
        get_package_share_directory("whill_navi2"),
        "config", "launch_arg", "full_data_path_launch_arg.yaml"
    )) as f:
        launcharg_full_data_path = yaml.safe_load(f)["full_data_path_launch"]

    file_names = list_files_in_directory(launcharg_full_data_path["rewaypoint_path_abs"])
    read_file_path = launcharg_full_data_path["waypoint_path_abs"]
    write_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
    read_file_name = launcharg_full_data_path["waypoint_name"]
    write_file_name = launcharg_full_data_path["rewaypoint_name"]
    for file_name in file_names:
        if file_names.count() == 0:
            read_file_path = launcharg_full_data_path["waypoint_path_abs"]
            write_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
            read_file_name = launcharg_full_data_path["waypoint_name"]
            write_file_name = launcharg_full_data_path["rewaypoint_name"]
        else:
            read_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
            write_file_path = read_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
            read_file_name = file_names[-1]
            write_file_name = str(file_names.count()) + launcharg_full_data_path["rewaypoint_name"]

    # Node
    waypoint_editor_rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="waypoint_editor_rviz2",
        arguments=["-f", os.path.join(
            get_package_share_directory("whill_navi2"),
            "config", "rviz2", "waypoint_editor_launch.rviz"
        )],
        output="screen"
    )
    waypoint_editor_node = Node(
        package="waypoint_pkg",
        executable="waypoint_editor",
        name="waypoint_editor",
        parameters=[{
            "read_file_name": os.path.join(
                read_file_path, read_file_name
            ),
            "write_file_name": os.path.join(
                write_file_path, write_file_name
            )
        }],
        output="screen"
    )
    
    # Lifecycle Node
    map_server_lifecycle_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="waypoint_editor_launch",
        parameters=[{
            "yaml_filename": os.path.join(
                launcharg_full_data_path["remap_path_abs"],
                launcharg_full_data_path["remap_name"]
            )
        }],
        output="screen"
    )
    
    # Event
    map_server_lifecycle_node_configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=waypoint_editor_rviz2_node,
            on_start=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )
    map_server_lifecycle_node_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_lifecycle_node,
            entities=[
                EmitEvent(
                    event=lifecycle.ChangeState(
                        lifecycle_node_matcher=matches_action(map_server_lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
                waypoint_editor_node
            ]
        )
    )
    shutdown_launch_event = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_editor_rviz2_node,
            on_exit=[Shutdown()]
        )
    )
    
    return LaunchDescription([        
        waypoint_editor_rviz2_node,
        map_server_lifecycle_node,
        map_server_lifecycle_node_configure_event,
        map_server_lifecycle_node_activate_event,
        shutdown_launch_event
    ])
