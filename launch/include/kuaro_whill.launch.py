import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    whill_model_xacro_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        'config',
        'xacro',
        'modelc.urdf.xacro'
    )
    whill_model_urdf_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        'config',
        'xacro',
        'modelc.urdf'
    )
    ros2_whill_yaml_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config',
        'params',
        'ros2_whill_params.yaml'
    )

    whill_model_urdf_file = xacro.process_file(whill_model_xacro_path)
    robot_description = whill_model_urdf_file.toprettyxml(indent='\t')
    with open(whill_model_urdf_path, "w") as urdf_file:
        urdf_file.write(robot_description)
    
    # Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[whill_model_urdf_path],                              # Nodeに与える引数、
        parameters=[{'robot_description': robot_description}],          # ROSのパラメータ
        remappings=[('/joint_states', '/whill/states/jointState')],     # トピックのremap
        output='screen'                                                 # ログをコンソール画面に出力する
    )
    ros2_whill_node = Node(
        package='ros2_whill',
        executable='ros2_whill',
        name='ros2_whill',
        output='screen',
        namespace='whill',
        parameters=[ros2_whill_yaml_path],
        remappings=[('whill/control/cmd_vel', 'cmd_vel')]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[('joy', 'whill/controller/joy')],
        output='screen'
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
