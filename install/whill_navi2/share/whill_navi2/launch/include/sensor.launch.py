import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'adis_imu_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_adis_imu = yaml.safe_load(f)['adis_imu_launch']
        
    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'ublox_gnss_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_ublox_gnss = yaml.safe_load(f)['ublox_gnss_launch']
        
    launcharg_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config', 'launch_arg', 'hokuyo_lrf_launch_arg.yaml'
    )
    with open(launcharg_path) as f:
        launcharg_hokuyo_lrf = yaml.safe_load(f)['hokuyo_lrf_launch']
        
        
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
        
    # Declare Launch Arguments
    use_adis_imu_arg = DeclareLaunchArgument("use_adis_imu", default_value="true")
    use_velodyne_arg = DeclareLaunchArgument("use_velodyne", default_value="true")
    use_ublox_arg = DeclareLaunchArgument("use_ublox", default_value="true")
    use_hokuyo_arg = DeclareLaunchArgument("use_hokuyo", default_value="true")
    use_web_camera_arg = DeclareLaunchArgument("use_web_camera", default_value="false")
    use_realsense_camera_arg = DeclareLaunchArgument("use_realsense_camera", default_value="false")
    use_zed_camera_arg = DeclareLaunchArgument("use_zed_camera", default_value="false")
                    
    # Include Launch file
    # IMU Launch
    adis_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'adis16465.launch.py'
            )
        ),
        launch_arguments=launcharg_adis_imu.items(),
        condition=IfCondition(LaunchConfiguration(use_adis_imu_arg.name))
    )
    # Ublox GNSS Launch
    ublox_gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'zed_f9p.launch.py'
            )
        ),
        launch_arguments=launcharg_ublox_gnss.items(),
        condition=IfCondition(LaunchConfiguration(use_ublox_arg.name))        
    )
    # Hokuyo LRF Launch
    hokuyo_lrf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'urg_node2.launch.py'
            )
        ),
        launch_arguments=launcharg_hokuyo_lrf.items(),
        condition=IfCondition(LaunchConfiguration(use_hokuyo_arg.name))        
    )
    # Web Camera Launch
    web_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'web_camera.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration(use_web_camera_arg.name))        
    )
    # Velodyne Launch
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('whill_navi2'),
                'launch', 'include', 'velodyne-all-nodes-VLP16-launch.py'
            )
        ),
        launch_arguments=[["with_rviz", "false"]],
        condition=IfCondition(LaunchConfiguration(use_velodyne_arg.name))        
    )
    
    # Group
    launch_group = GroupAction(actions=[
        velodyne_launch,
        web_camera_launch,
        hokuyo_lrf_launch,
        ublox_gnss_launch,
        adis_imu_launch
    ])
    
    return LaunchDescription([
        use_adis_imu_arg,
        use_velodyne_arg,
        use_ublox_arg,
        use_hokuyo_arg,
        use_web_camera_arg,
        use_realsense_camera_arg,
        use_zed_camera_arg,
        launch_group
    ])
    
    
# test
# if __name__ == '__main__':
#     generate_launch_description()
