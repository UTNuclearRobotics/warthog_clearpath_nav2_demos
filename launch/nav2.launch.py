# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch_ros.actions import Node


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path')
]


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')

    # Read robot YAML
    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        platform_model,
        'nav2.yaml'])

    launch_nav2 = PathJoinSubstitution(
      [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='cloud_to_scan',
            parameters=[{
                'use_sim_time': use_sim_time.perform(context) == 'true',
                'target_frame': 'lidar3d_0_sensor_link' if use_sim_time.perform(context) == 'true' else 'lidar3d_0_sensor_link',
                'min_height': -0.2,
                'max_height': 0.2,
                'range_min': 1.5,
            }],
            remappings=[
                ('cloud_in', '/' + namespace + '/sensors/lidar3d_0/points'),
                ('scan', '/' + namespace + '/sensors/lidar3d_0/scan1')
            ]
        ),
        # #TO filter out data if needed later on, make sure to tweak the nav2.yaml to use the filftered topic.
        # Node(
        #     package='laser_filters',  # Assuming the package is installed
        #     executable='laser_filter_node',
        #     name='laser_filter',
        #     parameters=[{
        #         'use_sim_time': use_sim_time.perform(context) == 'true',
        #         'range_min': 1.0,  # Minimum valid range
        #         'range_max': 30.0, # Maximum valid range
        #         'filter_method': 'range',  # Could be another method depending on your requirements
        #         'apply_smoothing': True,  # If you want to smooth the data
        #     }],
        #     remappings=[
        #         ('scan_in', '/' + namespace + '/sensors/lidar3d_0/scan1'),  # Remap input scan topic
        #         ('scan_out', '/' + namespace + '/sensors/lidar3d_0/scan_filtered')  # Filtered scan output
        #     ]
        # ),


        SetRemap('/' + namespace + '/global_costmap/sensors/lidar3d_0/scan1',
                 '/' + namespace + '/sensors/lidar3d_0/scan1'),
        SetRemap('/' + namespace + '/local_costmap/sensors/lidar3d_0/scan1',
                 '/' + namespace + '/sensors/lidar3d_0/scan1'),
        SetRemap('/' + namespace + '/odom',
                 '/' + namespace + '/platform/odom'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', file_parameters),
                ('use_composition', 'False'),
                ('namespace', namespace)
              ]
        ),
    ])

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
