from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():
    
    adis16465_params_yaml_path = get_yaml_path("whill_navi2", 'adis16465_node_params.yaml')

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    # IMUを起動する
    imu_node = Node(
        package="adi_driver",                                           # パッケージの名前
        executable="adis16465_node",
        name="adis16465_node",                                          # 実行ファイルの名前
        parameters=[adis16465_params_yaml_path,],
        remappings=[
            ("data_raw", "adis/imu/data_raw")
        ],
        output="screen"                                                 # ターミナルにLogを出力する
    )
    # IMUのデータフィルタを起動する
    imu_filter_node = Node(
        package="imu_filter_madgwick",                                  # パッケージの名前
        executable="imu_filter_madgwick_node",                          # 実行ファイルの名前
        name="adis_imu_filter_node",
        parameters=[adis16465_params_yaml_path],
        remappings=[
            ("imu/data_raw", "adis/imu/data_raw"),
            ("imu/data", "adis/imu/data"),
            ("imu/mag", "imu/mag")
        ]
    )

    return LaunchDescription([    
        imu_filter_node,
        imu_node
    ])