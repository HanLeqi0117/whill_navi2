import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            LogInfo, ExecuteProcess, RegisterEventHandler, TimerAction,
                            EmitEvent, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnShutdown, OnProcessExit
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events import lifecycle
from lifecycle_msgs.msg import Transition, TransitionEvent
from launch_ros.event_handlers import OnStateTransition

def list_files_in_directory(path):
    file_list = []
    for root, dirs, files in os.walk(path):
        for file in files:
            file_list.append(file)
    return sorted(file_list)  # ファイル名でソートしたリストを返す

def generate_launch_description():
    # ファイルを読み取りたいディレクトリのパスを指定
    directory_path = '/home/han/whill2_ws/full_data/2023_10_09/nakanoshima/rewaypoint'  # ディレクトリのパスを適切に置き換えてください

    if os.path.exists(directory_path) and os.path.isdir(directory_path):
        file_list = list_files_in_directory(directory_path)
        if file_list:
            for file_name in file_list:
                print(file_name)
        else:
            print("ディレクトリ内にファイルが見つかりません。")
    else:
        print("指定したディレクトリが存在しないか、ディレクトリではありません。")            
    return LaunchDescription([
        # slam_offline_rviz2_node,
        # map_saver_cli_lifecycle_node,
        # map_saver_cli_lifecycle_node_configure_event,
        # map_saver_cli_lifecycle_node_activate_event,
        # mv_map_to_remap_event
    ])
    
if __name__ == "__main__":
    generate_launch_description()