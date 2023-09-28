import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    ros2_whill_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros2_whill'), 'launch'),
            '/ros2_whill.launch.py'
        ]),
        launch_arguments={'use_gui': 'true',
                          'send_interval': '20',
                          'keep_connected': 'true', 
                          'serial_port': os.environ['TTY_WHILL'],
                          'publish_tf': 'true',
                          'enable_cmd_vel_topic': 'true'
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        remappings=[('joy', 'whill/controller/joy')],
        output='screen'
    )

    return LaunchDescription([
        ros2_whill_launch, 
        joy_node
    ])
