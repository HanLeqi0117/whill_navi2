import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    # xacro, urdf, yaml, rvizファイルのパス
    imu_urdf_file_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        "config",
        "urdf",
        "adis16470_breakout.urdf"
    )
    imu_rviz_file_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        "config",
        "rviz2",
        "adis_imu.rviz"
    )
    # ファイルを開き、内容を変数に納める
    with open(imu_urdf_file_path, "r") as imu_urdf_file:
            robot_description = imu_urdf_file.read()
            
            
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################

    # Launchの引数オブジェクトを宣言する
    with_filter_arg = DeclareLaunchArgument(name="with_filter", default_value="true")
    with_rviz_arg = DeclareLaunchArgument(name="with_rviz", default_value="true")
    with_plot_arg = DeclareLaunchArgument(name="with_plot", default_value="false")
    imu_device_arg = DeclareLaunchArgument(name="device", default_value="/dev/ttyACM0")
    frame_id_arg = DeclareLaunchArgument(name="frame_id", default_value="imu")
    burst_read_arg = DeclareLaunchArgument(name="burst_read", default_value="false")
    publish_temperature_arg = DeclareLaunchArgument(name="publish_temperature", default_value="false")
    rate_arg = DeclareLaunchArgument(name="rate", default_value="100.0")
    publish_tf_arg = DeclareLaunchArgument(name="publish_tf", default_value="true")
    publish_debug_topics_arg = DeclareLaunchArgument(name="publish_debug_topics", default_value="false")
    use_mag_arg = DeclareLaunchArgument(name="use_mag", default_value="false")
    raw_data_topic_arg = DeclareLaunchArgument(name="raw_data_topic", default_value="imu/data_raw")
    filtered_data_topic_arg = DeclareLaunchArgument(name="filtered_data_topic", default_value="imu/data")

    # Nodeのオブジェクトを宣言する
    # 起動するノードのオブジェクトの宣言
    # センサーのurdfファイルに基づいてRvizの中で可視化できるためにロボットの構造情報を送信する
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[imu_urdf_file_path],                                 # main関数の引数
        parameters=[{'robot_description': robot_description}],          # ROSのパラメータ
        output='screen'                                                 # ターミナルにLogを出力する
    )

    # IMUを起動する
    imu_node = Node(
        package="adi_driver",                                           # パッケージの名前
        executable="adis16465_node",                                    # 実行ファイルの名前
        name="adis16465_node",                                          # ノードの名前
        parameters=[{
            "device": LaunchConfiguration("device"),
             "frame_id": LaunchConfiguration("frame_id"),
             "burst_read": LaunchConfiguration("burst_read"),
             "publish_temperature": LaunchConfiguration("publish_temperature"),
             "rate": LaunchConfiguration("rate")
        }],
        remappings=[
            ("data_raw", LaunchConfiguration(raw_data_topic_arg.name))
        ],
        output="screen"                                                 # ターミナルにLogを出力する
    )
    # IMUのデータフィルタを起動する
    imu_filter_node = Node(
        package="imu_filter_madgwick",                                  # パッケージの名前
        executable="imu_filter_madgwick_node",                          # 実行ファイルの名前
        name="imu_filter_madgwick",                                     # ノードの名前
        parameters=[{                                                   # ノードのパラメータ
            "use_mag": LaunchConfiguration("use_mag"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "publish_debug_topics": LaunchConfiguration("publish_debug_topics")
        }],
        remappings=[
            ("imu/data_raw", LaunchConfiguration(raw_data_topic_arg.name)),
            ("imu/data", LaunchConfiguration(filtered_data_topic_arg.name)),
            ("imu/mag", "imu/mag")
        ],
        condition=IfCondition(LaunchConfiguration("with_filter"))       # ノードの実行条件
    )
    # Rvizを起動する
    rviz_node = Node(
        package="rviz2",                                                # パッケージの名前
        executable="rviz2",                                             # 実行ファイルの名前
        name="rviz2",                                                   # ノードの名前
        arguments=["-d", imu_rviz_file_path],                           # ターミナルの引数
        condition=IfCondition(LaunchConfiguration("with_rviz"))         # ノードの実行条件
    )
    # Plotを起動する(ジャイロセンサ情報表示)
    gyro_plot_node = Node(
        package="rqt_plot",                                             # パッケージの名前
        executable="rqt_plot",                                          # 実行ファイルの名前
        name="gyro",                                                    # ノードの名前
        namespace="plot",                                               # ノードのネームスペース
        arguments=["/imu/data_raw/angular_velocity/x:y:z"],             # ターミナルの引数
        condition=IfCondition(LaunchConfiguration("with_plot"))         # ノードの実行条件
    )

    # Plotを起動する(線加速度情報表示)
    accl_plot_node = Node(
        package="rqt_plot",                                             # パッケージの名前
        executable="rqt_plot",                                          # 実行ファイルの名前
        name="accl",                                                    # ノードの名前
        namespace="plot",                                               # ノードのネームスペース
        arguments=["/imu/data_raw/linear_accleration/x:y:z"],           # ターミナルの引数
        condition=IfCondition(LaunchConfiguration("with_plot"))         # ノードの実行条件
    )  
          
    # Node Group
    node_group = GroupAction(actions=[
        accl_plot_node,
        gyro_plot_node,
        rviz_node,
        imu_filter_node,
        imu_node,
        robot_state_publisher_node,
    ])

    return LaunchDescription([    
        with_filter_arg,
        raw_data_topic_arg,
        filtered_data_topic_arg,
        with_rviz_arg,
        with_plot_arg,
        imu_device_arg,
        frame_id_arg,
        burst_read_arg,
        publish_temperature_arg,
        rate_arg,
        publish_tf_arg,
        publish_debug_topics_arg,
        use_mag_arg,
        node_group  
    ])