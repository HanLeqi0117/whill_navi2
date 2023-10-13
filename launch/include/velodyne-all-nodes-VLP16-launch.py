# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, EmitEvent
import launch.event_handlers 
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    with_rviz_arg = DeclareLaunchArgument(name="with_rviz", default_value="true")

    rviz_file_dir = os.path.join(ament_index_python.packages.get_package_share_path("whill_navi2"), "config", "rviz2", "velodyne.rviz")
    velodyne_rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file_dir],
        condition=IfCondition(LaunchConfiguration(with_rviz_arg.name))
    )

    whill_navi2_config_params = ament_index_python.packages.get_package_share_directory('whill_navi2')
    driver_params_file = os.path.join(whill_navi2_config_params, 'config', 'params', 'VLP16-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file])

    convert_params_file = os.path.join(whill_navi2_config_params, 'config', 'params', 'VLP16-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(whill_navi2_config_params, 'config', 'params', 'VLP16db.yaml')
    velodyne_transform_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                      executable='velodyne_transform_node',
                                                      output='both',
                                                      parameters=[convert_params])

    laserscan_params_file = os.path.join(whill_navi2_config_params, 'config', 'params', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
                                                      executable='velodyne_laserscan_node',
                                                      output='both',
                                                      parameters=[laserscan_params_file])


    return launch.LaunchDescription([
        with_rviz_arg,
        velodyne_rviz_node,
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
        )),
    ])
