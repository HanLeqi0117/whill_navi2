import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    
    with open(os.path.join(
        get_package_share_directory("whill_navi2"),
        "config", "launch_arg", "full_data_path_launch_arg.yaml"
    )) as f:
        launcharg_full_data_path = yaml.safe_load(f)["full_data_path_launch"]
    with open(os.path.join(
        get_package_share_directory("whill_navi2"),
        "config", "launch_arg", "sensor_launch_arg.yaml"
    )) as f:
        launcharg_sensor = yaml.safe_load(f)["sensor_launch"]
    with open(os.path.join(
        get_package_share_directory("whill_navi2"),
        "config", "launch_arg", "kuaro_whill_launch_arg.yaml"
    )) as f:
        launcharg_kuaro_whill = yaml.safe_load(f)["kuaro_whill_launch"]
        
        
    # Node
    navigation_rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="navigation_rviz2_node",
        arguments=["-f", os.path.join(
            get_package_share_directory("whill_navi2"),
            "config", "rviz2", "navigation.rviz"
        )]
    )
    # Nodes
    tf2_static_nodes = GroupAction(actions=[
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_front_to_base_link",
            arguments = ['0.46', '0.12', '0.33', '0.0', '0.0', '3.14159', 'base_link', 'laser_front']
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_to_base_link",
            arguments = ['0.47', '0.46', '0.31', '-1.57079', '0.0', '0.0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_to_base_link',
            arguments = ['0.0', '0.29', '1.09', '0.0', '0.0', '0.0', 'base_link', 'velodyne']
        )
    ])
    
    # Lifecycle Node
    amcl_lifecycle_node = LifecycleNode(
        package="nav2_amcl",
        name="amcl",
        executable="amcl",
        namespace="navigation",
        parameters=[
            os.path.join(
                get_package_share_directory("whill_navi2"),
                "config", "params", "amcl_params.yaml"
            )
        ]
    )
    map_server_node = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filepath": os.path.join(
                launcharg_full_data_path["remap_path_abs"],
                launcharg_full_data_path["remap_name"]
            )
        }]
    )
        
    # Launch
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("whill_navi2"),
                "launch", "include", "sensor.launch.py"
            )
        ),
        launch_arguments=launcharg_sensor.items()
    )
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("whill_navi2"),
                "launch", "include", "kuaro_whill.launch.py"
            )
        ),
        launch_arguments=launcharg_kuaro_whill.items()
    )

    return LaunchDescription(
        
    )
    
