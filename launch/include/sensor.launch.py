import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node



def generate_launch_description():

    # Declare Launch Arguments
    use_adis_imu_arg = DeclareLaunchArgument("use_adis_imu", default_value="true")
    use_velodyne_arg = DeclareLaunchArgument("use_velodyne", default_value="true")
    use_ublox_arg = DeclareLaunchArgument("use_ublox", default_value="true")
    use_hokuyo_arg = DeclareLaunchArgument("use_hokuyo", default_value="true")
    use_web_camera_arg = DeclareLaunchArgument("use_web_camera", default_value="true")
    use_realsense_camera_arg = DeclareLaunchArgument("use_realsense_camera", default_value="true")
    use_zed_camera_arg = DeclareLaunchArgument("use_zed_camera", default_value="true")

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
                'launch', 'include', 'velodyne-all-nodes-VLP16-composed-launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration(use_velodyne_arg.name))        
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
# test
# if __name__ == '__main__':
#     generate_launch_description()
