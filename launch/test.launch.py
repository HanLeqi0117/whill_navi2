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

    file_names = list_files_in_directory(launcharg_full_data_path["bag_path_abs"])
    for file_name in file_names:
        if file_names.count() == 0:
            bag_path = launcharg_full_data_path["bag_file"]
        else:
            read_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
            write_file_path = read_file_path = launcharg_full_data_path["rewaypoint_path_abs"]
            read_file_name = file_names[-1]
            write_file_name = str(file_names.count()) + launcharg_full_data_path["rewaypoint_name"]

    return LaunchDescription([

    ])
    
if __name__ == "__main__":
    generate_launch_description()