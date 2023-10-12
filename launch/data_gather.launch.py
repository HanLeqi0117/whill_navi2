import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnExecutionComplete, OnProcessStart
from launch.conditions import IfCondition
from launch_ros.actions import Node
from datetime import date

def generate_launch_description():
    
    # Set Launch Arguments
    use_gnss_localization_arg =  DeclareLaunchArgument("use_gnss_localization", default_value="false")
    
    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'sensor_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_sensor = yaml.safe_load(f)['sensor_launch']
    
    paramspath_make_dir = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'params', 'make_dir_node_params.yaml'
    ) 
    with open(paramspath_make_dir) as f:
        nodeparams_make_dir = yaml.safe_load(f)['make_dir_node']['ros__parameters']
            
    #Include Launch File
    #sensor Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'sensor.launch.py'
            )
        ),
        launch_arguments=launcharg_sensor.items()
    )
    #kuaro_whill Launch
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'kuaro_whill.launch.py'
            )
        )
    )
    
    # Nodes
    # データ保存用ディレクトリ作成用Node
    # Path_Tree
    # 例：
    # $HOME/$WORKSPACE
    # └── full_data
    #     └── date_today
    #         └── nakanoshima
    #             ├── branchpoint
    #             ├── data_gather_bag
    #             ├── finalwaypoint
    #             ├── map
    #             ├── production_bag
    #             ├── remap
    #             ├── rewaypoint
    #             └── waypoint
    make_dir_node = Node(
        package='whill_navi2',
        executable='make_dir_node',
        name='make_dir_node',
        parameters=[paramspath_make_dir]
    )

    # Publish LRF static transforms
    tf2_static_hokuyo_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_front_to_base_link',
        arguments = [
            '--x', '0.46', '--y', '0.12', '-z', '0.33', 
            '--roll', '0.0', '--ptich', '0.0', '--yaw', '3.14159', 
            '--frame-id', 'base_link', '--child-frame-id', 'laser_front'
        ]  
    )
    # Publish Velodyne static transforms
    tf2_static_velodyne_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link',
        arguments = [
            '--x', '0.0', '--y', '0.29', '-z', '1.09', 
            '--roll', '0.0', '--ptich', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'velodyne'
        ]  
    )
   
    # Publish IMU static transforms
    tf2_static_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments = [
            '--x', '0.47', '--y', '0.46', '-z', '0.31', 
            '--roll', '-1.57079', '--ptich', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
        ] 
    )

   # Publish gnss static transforms
    tf2_static_gnss_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_to_base_link',
        arguments = [
            '--x', '0.71', '--y', '0.24', '-z', '0.12', 
            '--roll', '0.0', '--ptich', '0.0', '--yaw', '0.0', 
            '--frame-id', 'base_link', '--child-frame-id', 'gnss'
        ]
    )

    # robot_localization pkg EKF odometry
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
    # ros2bag
    ros2bag_record_process = ExecuteProcess(
        cmd=[FindExecutable(name='ros2'),
            'bag', 'record', '--all', '-o', 
            os.path.join(
                os.environ['HOME'],
                nodeparams_make_dir['ws_path'],
                "full_data",
                str(nodeparams_make_dir['date_path']),
                nodeparams_make_dir['place_path'],
                nodeparams_make_dir["bag_path"]
            )
        ]
    )
    ros2bag_record_event = RegisterEventHandler(
        OnProcessStart(
            target_action=make_dir_node,
            on_start=[
                LogInfo(msg='Sensors are launched, and then start to record the data.'),
                TimerAction(
                    actions=[
                        ros2bag_record_process],
                    period=1.0
                )
            ]
        )
    )
    
    #rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='datagather_rviz2_node',
        arguments=['-d', os.path.join(
                get_package_share_directory('whill_navi2'), 
                'config', 'rviz2', 'data_gather_launch.rviz'
            )
        ]
    )

    node_group = GroupAction(actions=[
        make_dir_node,
        tf2_static_imu_node,
        tf2_static_gnss_node,
        tf2_static_hokuyo_node,
        tf2_static_velodyne_node,
        ekf_odometry_node,
        rviz2_node
    ])

    launch_group = GroupAction(
        actions=[
            sensor_launch,
            kuaro_whill_launch
        ]
    )

    return LaunchDescription([        
        ros2bag_record_event,
        node_group,
        launch_group
    ])
