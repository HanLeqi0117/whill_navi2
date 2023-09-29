import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Set Launch Arguments
    ws_path_arg = DeclareLaunchArgument("ws_path", default_value="whill2_ws")
    place_path_arg = DeclareLaunchArgument("place_path", default_value="nakanoshima")
    data_path_arg = DeclareLaunchArgument("data_path", default_value="2023_07_21")
    bagfile_path_arg = DeclareLaunchArgument("bagfile_path", default_value="bag")
    map_path_arg = DeclareLaunchArgument("map_path", default_value="map")
    waypoint_path_arg = DeclareLaunchArgument("waypoint_path", default_value="waypoint")
    branchpoint_path_arg = DeclareLaunchArgument("branchpoint_path", default_value="branchpoint")
    bagfile_name_arg = DeclareLaunchArgument("bagfile_name", default_value="bag_test")
    
    # Include Launchs
    # WHILLのドライバを起動するLaunch
    # 引数1：Launchファイルのアドレス
    kuaro_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch',
                'include',
                'kuaro_whill.launch.py'
            )
        )
    )

    # Nodes
    # データ保存用ディレクトリ作成用Node
    # Path_Tree
    # 例：
    #     nakanoshima/
    #     └── 2023_07_21
    #         ├── branchpoint
    #         ├── data_gather_bagfile
    #         ├── finalwaypoint
    #         ├── map
    #         ├── production_bagfile
    #         ├── remap
    #         ├── rewaypoint
    #         └── waypoint
    make_dir_node = Node(
        package='make_dir',
        executable='make_dir_node',
        name='make_dir_node',
        parameters=[{
            "ws_path": LaunchConfiguration("ws_path"),
            "place_path": LaunchConfiguration("place_path"),
            "data_path": LaunchConfiguration("data_path"),
            "map_path": LaunchConfiguration("bagfile_path"),
            "bagfile_path": LaunchConfiguration("map_path"),
            "waypoint_path": LaunchConfiguration("waypoint_path"),
            "branchpoint_path": LaunchConfiguration("branchpoint_path")
        }]
    )

    return LaunchDescription([

    ])
