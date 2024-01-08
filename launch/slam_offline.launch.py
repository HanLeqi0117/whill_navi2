from whill_navi2.modules.ros2_launch_utils import *

def generate_launch_description():

    slam_params_yaml_path = get_yaml_path("whill_navi2", "slam_toolbox_params.yaml")
    data_path = DataPath()
    
##############################################################################################
####################################### ROS LAUNCH API #######################################
##############################################################################################
    
    # Node Action
    slam_offline_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='slam_offline_rviz2',
        arguments=['-d', get_rviz_path("whill_navi2", "slam_offline_launch.rviz")]
    )  
    slam_offline_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',    
        name='slam_toolbox',
        parameters=[slam_params_yaml_path],
        output='screen'
    )
    map_saver_cli_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        arguments=['-f', data_path.map_path]
    )
    cp_map2remap_node = Node(
        package="whill_navi2",
        executable='map2remap_node',
        output="screen"
    )
    
    # Process Action
    bag_play_process = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", data_path.bag_path, "--rate=0.6"
        ]
    )
    
    # Event Action
    when_slam_start = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_offline_rviz2_node,
            on_start=[
                slam_offline_node,
                TimerAction(
                    period=0.5,
                    actions=[bag_play_process]
                )
            ]
        )
    )
    when_bag_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=bag_play_process,
            on_completion=[
                LogInfo(msg="MapSaver is starting..."),
                TimerAction(
                    period=0.5,
                    actions=[map_saver_cli_node]
                )
            ]
        )
    )
    when_map_saver_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=map_saver_cli_node,
            on_completion=[
                TimerAction(
                    period=0.1,
                    actions=[cp_map2remap_node]
                )
            ]
        )
    )
    when_cp_over = RegisterEventHandler(
        OnExecutionComplete(
            target_action=cp_map2remap_node,
            on_completion=[
                Shutdown()
            ]
        )
    )

    return LaunchDescription([
        slam_offline_rviz2_node,
        when_slam_start,
        when_bag_over,
        when_map_saver_over,
        when_cp_over
    ])


# test
# if __name__ == '__main__':
#     generate_launch_description()