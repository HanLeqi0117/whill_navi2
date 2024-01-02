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

class DataPath:
    bag_name = bag_dir = branchpoint_dir = \
    date_dir = map_name = map_dir = \
    place_dir = remap_name = remap_dir = \
    rewaypoint_name = rewaypoint_dir = waypoint_name = \
    waypoint_dir = ws_dir = backup_bag_dir = ''
    
    bag_path = map_path = remap_map = \
    waypoint_path = rewaypoint_path = ''
        
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

def get_node_params_dict(package_name = str, yaml_file_name = str, node_name = str):
    
    yaml_path = get_yaml_path(package_name, yaml_file_name)
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

def get_data_path():
    
    mkdir_node_params_dict = get_node_params_dict('whill_navi2', 'make_dir_node_params.yaml', 'make_dir_node')
    base_dir = os.path.join(
        os.environ['HOME'],
        mkdir_node_params_dict['ws_dir'],
        'full_data',
        str(mkdir_node_params_dict['date_dir']),
        mkdir_node_params_dict['place_dir']
    )

    data_path = DataPath()
    data_path.bag_dir = os.path.join(base_dir, mkdir_node_params_dict['bag_dir'])
    data_path.backup_bag_dir = os.path.join(base_dir, 'backup_bag')
    data_path.map_dir = os.path.join(base_dir, mkdir_node_params_dict['map_dir'])
    data_path.remap_dir = os.path.join(base_dir, mkdir_node_params_dict['remap_dir'])
    data_path.waypoint_dir = os.path.join(base_dir, mkdir_node_params_dict['waypoint_dir'])
    data_path.rewaypoint_dir = os.path.join(base_dir, mkdir_node_params_dict['rewaypoint_dir'])
    
    data_path.bag_name = mkdir_node_params_dict['bag_name']
    data_path.map_name = mkdir_node_params_dict['map_name']
    data_path.remap_name = mkdir_node_params_dict['remap_name']
    data_path.waypoint_name = mkdir_node_params_dict['waypoint_name']
    data_path.rewaypoint_name = mkdir_node_params_dict['rewaypoint_name']
    
    data_path.bag_path = os.path.join(data_path.bag_dir, data_path.bag_name)
    data_path.map_path = os.path.join(data_path.map_dir, data_path.map_name)
    data_path.remap_map = os.path.join(data_path.remap_dir, data_path.remap_name)
    data_path.waypoint_path = os.path.join(data_path.waypoint_dir, data_path.waypoint_name)
    data_path.rewaypoint_path = os.path.join(data_path.rewaypoint_dir, data_path.rewaypoint_name)
    
    return data_path

def backup_bagfile():
    data_path = get_data_path()
    
    if os.path.exists(data_path.bag_dir):
        bag_name_list = list_files_in_directory(data_path.bag_dir)
        if len(bag_name_list) > 0:
            for bag_name in bag_name_list:
                if bag_name == data_path.bag_name:
                    backup_bag_file = bag_name + '_{:%Y_%m_%d_%H_%M_%S}'.format(datetime.datetime.now())
                    if os.path.exists(data_path.backup_bag_dir):
                        shutil.move(
                            os.path.join(data_path.bag_dir, data_path.bag_name),
                            os.path.join(data_path.backup_bag_dir, backup_bag_file)
                        )
                    else:
                        os.makedirs(data_path.backup_bag_dir)
                        shutil.move(
                            os.path.join(data_path.bag_dir, data_path.bag_name),
                            os.path.join(data_path.backup_bag_dir, backup_bag_file)
                        )

