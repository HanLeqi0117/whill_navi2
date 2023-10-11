import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, TimerAction
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
    tf2_static_LRF_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_front_to_base_link',
        arguments = ['0.46', '0.12', '0.33', '0.0', '0.0', '3.14159', 'base_link', 'laser_front']
    )
    # Publish Velodyne static transforms
    tf2_static_Velodyne_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_to_base_link',
        arguments = ['0.0', '0.29', '1.09', '0.0', '0.0', '0.0', 'base_link', 'velodyne']
    )
    # Publish IMU static transforms
    tf2_static_IMU_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_link',
        arguments = ['0.47', '0.46', '0.31', '-1.57079', '0.0', '0.0', 'base_link', 'imu_link']
    )
    # Publish gnss static transforms
    tf2_static_IMU_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_to_base_link',
        arguments = ['0.71', '0.24', '0.12', '0.0', '0.0', '0.0', 'base_link', 'imu_link']
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
                str(nodeparams_make_dir['date_path']),
                nodeparams_make_dir['place_path'],
                nodeparams_make_dir['bag_path'],
                nodeparams_make_dir['bag_name'],
            )
        ]
    )
    ros2bag_record_event = RegisterEventHandler(
        OnExecutionComplete(
            target_action=make_dir_node,
            on_start=[
                LogInfo(msg='Sensors are launched, and then start to record the data.'),
                ros2bag_record_process
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

    name_dict = locals()
    value_list = []
    for name, value in name_dict.items():
        if ("_arg") in name \
        or ("_node") in name \
        or ("_launch") in name \
        or ("_event") in name:
            # test
            # print(name, type(value))
            value_list.append(value)
            
    return LaunchDescription(
        value_list
    )
