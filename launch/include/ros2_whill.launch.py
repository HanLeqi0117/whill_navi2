
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    # このメソッドの名前は実行に影響がないが、このメソッドの戻り値はLaunchDescriptionオブジェクトであるべき

    # xacro, urdf, yamlファイルパスを記述する
    # modelc.urdf.xacroの内容は、XML言語でWHILLのモデルを記述するものである
    whill_model_xacro_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        'config',
        'xacro',
        'modelc.urdf.xacro'
    )

    # .xacroファイルはマクロ変数が入っているため、直接にロボットパブリッシュのパッケージに読まれることができないため、.xacroファイルから.urdfファイルに変換する必要があります。
    whill_model_urdf_path = os.path.join(
        get_package_share_directory("whill_navi2"),
        'config',
        'xacro',
        'modelc.urdf'
    )

    # .yamlファイルの内容は、XML言語でWHILLのパラメータを記述するものである
    ros2_whill_yaml_path = os.path.join(
        get_package_share_directory('whill_navi2'),
        'config',
        'params',
        'initial_speedprofile.yaml'
    )
    

    # xacroファイルに基づき、urdfのドキュメントオブジェクトを生成する
    whill_model_urdf_doc = xacro.process_file(whill_model_xacro_path)
    # urdfのドキュメントオブジェクトのメソッドtoprettyxmlを使用し、urdf内容を含むxml形式の文字列を生成する
    robot_description = whill_model_urdf_doc.toprettyxml(indent='\t')
    # urdfファイルをopen操作する
    urdf_file = open(whill_model_urdf_path, "w")
    # urdf内容を含む文字列をファイルに書き込む
    urdf_file.write(robot_description)
    # ファイルを閉じる
    urdf_file.close()

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Launch引数オブジェクトを宣言する、引数1： パラメータの名、引数2： パラメータの値
    # Launchファイルの引数は他のLaunchファイルに包括された場合、このLaunchファイルを包括するLaunchファイルで引数を修正すると、このLaunchファイルで反映される
    gui_arg = DeclareLaunchArgument('use_gui', default_value='true')
    # 送信の時間間隔
    send_interval_arg = DeclareLaunchArgument('send_interval', default_value='20')
    # パケットの送受信が途切れた場合、シリアル通信を維持するかどうか。Trueの場合、維持する。Falseの場合、維持しない。
    keep_connected_arg = DeclareLaunchArgument('keep_connected', default_value='true')
    # TTYファイルのアドレス
    # TTYファイルについて、URL：https://wa3.i-3-i.info/word11668.html
    # ここで、「~/.bashrc」ファイルで「TTY_WHILL」という環境変数を設定したので、環境変数を取得する。
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0')
    # ROSの座標変換情報を送信するかどうか。Trueの場合、送信する。Falseの場合、送信しない。
    publish_tf_arg = DeclareLaunchArgument('publish_tf', default_value='true')
    # ROSのトピック「cmd_vel」を受信するかどうか。Trueの場合、受信する。Falseの場合、受信しない。
    # 受信する場合、このトピックに速度指令を送信するだけで、WHILLに動かせる。
    enable_cmd_vel_topic_arg = DeclareLaunchArgument('enable_cmd_vel_topic', default_value='true')
    odometry_topic_name_arg = DeclareLaunchArgument('odometry_topic_name', default_value='odometry')
    
    
    
    # 起動するノードのオブジェクトの宣言
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
        parameters=[
            ros2_whill_yaml_path, 
            {'send_interval': LaunchConfiguration('send_interval'),    # LaunchConfigurationクラスは、定義されたLaunchArgumentの値を取得するものである
             'keep_connected': LaunchConfiguration('keep_connected'), 
             'serial_port': LaunchConfiguration('serial_port'),
             'publish_tf': LaunchConfiguration('publish_tf'),
             'enable_cmd_vel_topic': LaunchConfiguration('enable_cmd_vel_topic'),
             'odometry_topic_name': LaunchConfiguration('odometry_topic_name')}.items()
        ],
        remappings=[('whill/controller/cmd_vel', 'cmd_vel')]
    )

    # Group
    arg_group = GroupAction(actions=[
        gui_arg,
        send_interval_arg,
        keep_connected_arg,
        serial_port_arg,
        publish_tf_arg,
        enable_cmd_vel_topic_arg,
        odometry_topic_name_arg
    ])
    node_group = GroupAction(actions=[
        robot_state_publisher_node,
        ros2_whill_node,
        
    ])    
            
    return LaunchDescription([
        arg_group,
        node_group
    ])
