import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
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
    whill_joy2_params_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config',
        'params',
        'whill_joy2_params.yaml'
    )
    whill_model_urdf_file = xacro.process_file(whill_model_xacro_path)
    robot_description = whill_model_urdf_file.toprettyxml(indent='\t')
    with open(whill_model_urdf_path, "w") as urdf_file:
        urdf_file.write(robot_description)    
        
        
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[whill_model_urdf_path],                              # Nodeに与える引数、
        parameters=[{'robot_description': robot_description}],          # ROSのパラメータ
        remappings=[('joint_states', 'whill/states/jointState')],     # トピックのremap
        output='screen'                                                 # ログをコンソール画面に出力する
    )
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    ros2_whill_node = Node(
        package='ros2_whill',
        executable='ros2_whill',
        name='ros2_whill',
        output='screen',
        namespace='whill',
        parameters=[ros2_whill_yaml_path]
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[
            ("joy", "whill/controller/joy")
        ],
        output='screen'
    )
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     namespace='joy_node',
    #     output='screen'
    # )
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    # whill_joy2_node = Node(
    #     package='ros2_whill',
    #     executable='whill_joy2',
    #     name='whill_joy2',
    #     output='screen',
    #     parameters=[whill_joy2_params_path],
    #     remappings=[
    #         ('joy_state', 'joy_node/joy'),
    #         ('controller/joy', 'whill/controller/joy'),
    #         ('controller/cmd_vel', 'vel_to_joy/cmd_vel')
    #     ]
    # )
    
    node_group = GroupAction(actions=[
        robot_state_publisher_node,
        ros2_whill_node,
        joy_node,
        # whill_joy2_node
    ])   
            
    return LaunchDescription([
        node_group
    ])
