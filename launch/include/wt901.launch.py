from whill_navi2.ros2_launch_utils import *

def generate_launch_description():
    
    ld = LaunchDescription()
        
    wit901_node=Node(
        package = 'witmotion_ros',
        executable = 'witmotion_ros_node',
        parameters = [
            get_node_params_dict("whill_navi2", "wt901.yaml", "witmotion_ros_node")
        ]
    )
    # IMUのデータフィルタを起動する
    imu_filter_node = Node(
        package="imu_filter_madgwick",                                  # パッケージの名前
        executable="imu_filter_madgwick_node",                          # 実行ファイルの名前
        name="wit_imu_filter_node",
        parameters = [
            get_node_params_dict("whill_navi2", "wt901.yaml", "wit_imu_filter_node")
        ],
        remappings=[
            ("imu/data_raw", "witmotion/imu/data_raw"),
            ("imu/data", "witmotion/imu/data"),
            ("imu/mag", "magnetometer")
        ]
    )

    ld.add_action(wit901_node)
    ld.add_action(imu_filter_node)
    
    return ld