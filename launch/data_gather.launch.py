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
        'config', 'launch_arg', 'kuaro_whill_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_kuaro_whill = yaml.safe_load(f)['kuaro_whill_launch']
    
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
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'kuaro_whill.launch.py'
            )
        ),
        launch_arguments=launcharg_kuaro_whill.items()
    )
    
    # Node
    make_dir_node = Node(
        package='whill_navi2',
        executable='make_dir_node',
        name='make_dir_node',
        parameters=[paramspath_make_dir]
    )
    tf2_static_hokuyo_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_front_to_base_link',
        arguments = [
            '--x', '0.50', '--y', '0.0', '--z', '0.33', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159', 
            '--frame-id', 'base_link', '--child-frame-id', 'laser_front'
        ]  
    )
    tf2_static_velodyne_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link',
        arguments = [
            '--x', '0.0', '--y', '0.0', '--z', '0.8875', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'velodyne'
        ]  
    )
    tf2_static_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments = [
            '--x', '0.44', '--y', '0.23', '--z', '0.2575', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'imu'
        ]
        # IMU TF Arguments
        # arguments = [
        #     '--x', '0.44', '--y', '0.23', '--z', '0.2575', 
        #     '--roll', '-1.57079', '--pitch', '0.0', '--yaw', '0.0', 
        #     '--frame-id', 'base_link', '--child-frame-id', 'imu'
        # ]        
    )
    tf2_static_gnss_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_to_base_link',
        arguments = [
            '--x', '0.71', '--y', '0.0', '--z', '0.0175', 
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'gnss'
        ]
    )
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
    ros2bag_backup_process = ExecuteProcess(
        cmd=[
            "cp", "-r", 
            os.path.join(bag_path, nodeparams_make_dir['bag_name']),
            os.path.join(
                bag_path, '..', 
                'backup_bag',
                'backup_bag' + 
                    '_{:%Y_%m_%d_%H_%M_%S}'.format(datetime.datetime.now())
            )
        ]
    )

    # Launch Group
    launch_group = GroupAction(actions=[
        sensor_launch,
        kuaro_whill_launch
    ])
    # Node Group
    node_group = GroupAction(actions=[
        make_dir_node,
        tf2_static_hokuyo_node,
        tf2_static_velodyne_node,
        tf2_static_imu_node,
        tf2_static_gnss_node,
        ekf_odometry_node,
        rviz2_node
    ])
    # Action Group
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
    
    # Event Group
    event_group = GroupAction(actions=[
        ros2bag_record_event,
    ])

    return LaunchDescription([   
        action_group, 
        event_group
    ])
