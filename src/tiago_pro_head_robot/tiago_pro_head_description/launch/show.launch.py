# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_head_description.launch_arguments import TiagoProHeadArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    camera_model: DeclareLaunchArgument = TiagoProHeadArgs.camera_model


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='tiago_pro_head_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={"namespace": launch_args.namespace,
                          "use_sim_time": launch_args.use_sim_time,
                          "camera_model": launch_args.camera_model,
                          })

    launch_description.add_action(robot_state_publisher)

    joint_state_pub_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')

    launch_description.add_action(joint_state_pub_gui)

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('tiago_pro_head_description'), 'config', 'show.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')
                     }])
    launch_description.add_action(rviz)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
