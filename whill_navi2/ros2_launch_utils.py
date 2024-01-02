import os, sys, rclpy, shutil, ruamel.yaml, datetime

# Ros Launch Modules
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnExecutionComplete, OnProcessStart, OnShutdown
from launch.conditions import IfCondition
from launch_ros.actions import Node
        
# input: path of directory.
# output: list of the directory names in the path
def list_files_in_directory(path=str):
    dir_list = []
    for root, dirs, files in os.walk(path):
        for dir in dirs:
            dir_list.append(dir)
    return sorted(dir_list)

# input: package_name and yaml_file_name
# output: the dictionary of parameters
def get_yaml_path(package_name = str, yaml_file_name = str):
    return os.path.join(
        get_package_share_directory(package_name),
        'config', 'params', yaml_file_name
    )

def get_node_params_dict(yaml_path = str, node_name = str):
    with open(yaml_path) as f:
        data = ruamel.yaml.safe_load(f)[node_name]["ros__parameters"]
    return data


def get_include_launch_path(package_name = str, launch_filename = str):
    return os.path.join(
        get_package_share_directory(package_name),
        "launch", "include", launch_filename
    )
    
def get_urdf_path(package_name = str, urdf_filename = str):
    return os.path.join(
        get_package_share_directory(package_name),
        "config", "urdf", urdf_filename
    )
    
def get_rviz_path(package_name = str, rviz_filename = str):
    return os.path.join(
        get_package_share_directory(package_name),
        "config", "rviz2", rviz_filename
    )


class DataPath:
    
    def __init__(self):
        mkdir_params_yaml_path = get_yaml_path('whill_navi2', 'make_dir_node_params.yaml')
        mkdir_node_params_dict = get_node_params_dict(mkdir_params_yaml_path, 'make_dir_node')
        base_dir = os.path.join(
            os.environ['HOME'],
            mkdir_node_params_dict['ws_dir'],
            'full_data',
            str(mkdir_node_params_dict['date_dir']),
            mkdir_node_params_dict['place_dir']
        )

        self.bag_dir = os.path.join(base_dir, mkdir_node_params_dict['bag_dir'])
        self.backup_bag_dir = os.path.join(base_dir, 'backup_bag')
        self.map_dir = os.path.join(base_dir, mkdir_node_params_dict['map_dir'])
        self.remap_dir = os.path.join(base_dir, mkdir_node_params_dict['remap_dir'])
        self.waypoint_dir = os.path.join(base_dir, mkdir_node_params_dict['waypoint_dir'])
        self.rewaypoint_dir = os.path.join(base_dir, mkdir_node_params_dict['rewaypoint_dir'])
        
        self.bag_name = mkdir_node_params_dict['bag_name']
        self.map_name = mkdir_node_params_dict['map_name']
        self.remap_name = mkdir_node_params_dict['remap_name']
        self.waypoint_name = mkdir_node_params_dict['waypoint_name']
        self.rewaypoint_name = mkdir_node_params_dict['rewaypoint_name']
        
        self.bag_path = os.path.join(self.bag_dir, self.bag_name)
        self.map_path = os.path.join(self.map_dir, self.map_name)
        self.remap_map = os.path.join(self.remap_dir, self.remap_name)
        self.waypoint_path = os.path.join(self.waypoint_dir, self.waypoint_name)
        self.rewaypoint_path = os.path.join(self.rewaypoint_dir, self.rewaypoint_name)

    def backup_bagfile(self):
        if os.path.exists(self.bag_dir):
            bag_name_list = list_files_in_directory(self.bag_dir)
            if len(bag_name_list) > 0:
                for bag_name in bag_name_list:
                    if bag_name == bag_name:
                        backup_bag_file = bag_name + '_{:%Y_%m_%d_%H_%M_%S}'.format(datetime.datetime.now())
                        if os.path.exists(self.backup_bag_dir):
                            shutil.move(
                                os.path.join(self.bag_dir, bag_name),
                                os.path.join(self.backup_bag_dir, backup_bag_file)
                            )
                        else:
                            os.makedirs(self.backup_bag_dir)
                            shutil.move(
                                os.path.join(self.bag_dir, bag_name),
                                os.path.join(self.backup_bag_dir, backup_bag_file)
                            )

