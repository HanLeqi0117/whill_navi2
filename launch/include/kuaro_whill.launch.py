from whill_navi2.ros2_launch_utils import *

def generate_launch_description():
    
    whill_params_data = get_node_params_dict("whill_navi2", "ros2_whill_params.yaml", "whill/ros2_whill")
    joy_params_data = get_node_params_dict("whill_navi2", "ros2_whill_params.yaml", "whill_joy2")

##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    # Argument
    use_joycon_arg = DeclareLaunchArgument(name="use_joycon", default_value='true')
    
    # Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',                                # パッケージの名前
        executable='robot_state_publisher',                             # 実行ファイルの名前
        name='robot_state_publisher',                                   # ノードの名前
        arguments=[get_urdf_path("whill_navi2", "modelc.urdf")],        # Nodeに与える引数、
        remappings=[('joint_states', 'whill/states/jointState')],       # トピックのremap
        output='screen'                                                 # ログをコンソール画面に出力する
    )
    # Parameter YAML file: config/param/ros2_whill_params.yaml
    ros2_whill_node = Node(
        package='ros2_whill',
        executable='ros2_whill',
        name='ros2_whill',
        output='screen',
        namespace='whill',
        remappings=[
            ("odom", "odometry")
        ],
        parameters=[whill_params_data]
    )
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     remappings=[
    #         ("joy", "whill/controller/joy")
    #     ],
    #     output='screen'
    # )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    # Parameter YAML file: config/param/whill_joy2_params.yaml
    whill_joy2_node = Node(
        package='ros2_whill',
        executable='whill_joy2',
        output='screen',
        parameters=[
            joy_params_data,
            {"use_joycon" : LaunchConfiguration("use_joycon")}
        ],
        remappings=[
            ('joy_state', 'joy'),
            ('controller/joy', 'whill/controller/joy'),
            ('controller/cmd_vel', 'vel_to_joy/cmd_vel')
        ]
    )
    
    node_group = GroupAction(actions=[
        robot_state_publisher_node,
        ros2_whill_node,
        joy_node,
        whill_joy2_node
    ])
            
    return LaunchDescription([
        use_joycon_arg,
        node_group
    ])
