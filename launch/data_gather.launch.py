import os
import yaml
import shutil
import datetime

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
def list_files_in_directory(path):
    dir_list = []
    for root, dirs, files in os.walk(path):
        for dir in dirs:
            dir_list.append(dir)
    return sorted(dir_list)

def generate_launch_description():

    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'sensor_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_sensor = yaml.safe_load(f)['sensor_launch']
            
    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'tf2_static_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_tf2_static = yaml.safe_load(f)['tf2_static_launch']    
    
    paramspath_make_dir = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'params', 'make_dir_node_params.yaml'
    ) 
    with open(paramspath_make_dir) as f:
        nodeparams_make_dir = yaml.safe_load(f)['make_dir_node']['ros__parameters']
    
    bag_path = os.path.join(
        os.environ['HOME'],
        nodeparams_make_dir['ws_path'],
        'full_data',
        str(nodeparams_make_dir['date_path']),
        nodeparams_make_dir['place_path'],
        nodeparams_make_dir['bag_path']
    )
    
    # if bag_file in bag_path, it will take a backup
    if os.path.exists(bag_path):
        bag_files_list = list_files_in_directory(bag_path)
        if len(bag_files_list) > 0:
            for bag_file in bag_files_list:
                backup_bag_path = os.path.join(bag_path, '..', 'backup_bag')
                backup_bag_file = bag_file + '_{:%Y_%m_%d_%H_%M_%S}'.format(datetime.datetime.now())
                if os.path.exists(backup_bag_path):
                    shutil.move(
                        os.path.join(bag_path, bag_file),
                        os.path.join(backup_bag_path, backup_bag_file)
                    )
                else:
                    os.makedirs(backup_bag_path)
                    shutil.move(
                        os.path.join(bag_path, bag_file),
                        os.path.join(backup_bag_path, backup_bag_file)
                    )
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Include Launch File
    # sensor Launch
    # Argument YAML file: config/launch_arg/sensor_launch_arg.yaml
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'sensor.launch.py'
            )
        ),
        launch_arguments=launcharg_sensor.items()
    )
    # kuaro_whill Launch
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'kuaro_whill.launch.py'
            )
        )
    )
    # tf2_static Launch
    # Argument YAML file: config/launch_arg/tf2_static_launch_arg.yaml
    tf2_static_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'tf2_static.launch.py'
            )
        ),
        launch_arguments=launcharg_tf2_static.items()
    )
    
    # Node
    # Parameter YAML file: config/param/make_dir_node_params.yaml
    make_dir_node = Node(
        package='whill_navi2',
        executable='make_dir_node',
        name='make_dir_node',
        parameters=[paramspath_make_dir]
    )
    # Parameter YAML file: config/param/ekf_node_params.yaml
    ekf_odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'config', 'params', 'ekf_node_params.yaml'
            )
        ]
    )
    # Rviz config file: config/rviz2/data_gather_launch.rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='datagather_rviz2_node',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('whill_navi2'), 
                'config', 'rviz2', 'data_gather_launch.rviz'
            )
        ]
    )
    
    # Process
    ros2bag_record_process = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'bag', 'record', '--all', '-o', 
            os.path.join(
                os.environ['HOME'],
                nodeparams_make_dir['ws_path'],
                "full_data",
                str(nodeparams_make_dir['date_path']),
                nodeparams_make_dir['place_path'],
                nodeparams_make_dir["bag_path"],
                nodeparams_make_dir["bag_name"]
            )
        ]
    )

    ## Group
    # Launch
    launch_group = GroupAction(actions=[
        sensor_launch,
        kuaro_whill_launch,
        tf2_static_launch
    ])
    # Node
    node_group = GroupAction(actions=[
        make_dir_node,
        ekf_odometry_node,
        rviz2_node
    ])
    # Action
    action_group = GroupAction(actions=[
        node_group,
        launch_group
    ])

    # Event
    ros2bag_record_event = RegisterEventHandler(
        OnProcessStart(
            target_action=make_dir_node,
            on_start=[
                LogInfo(msg='Sensors are launched, and then start to record the data.'),
                TimerAction(
                    actions=[ros2bag_record_process],
                    period=0.5
                )
            ]
        )
    )

    return LaunchDescription([   
        action_group, 
        ros2bag_record_event
    ])
