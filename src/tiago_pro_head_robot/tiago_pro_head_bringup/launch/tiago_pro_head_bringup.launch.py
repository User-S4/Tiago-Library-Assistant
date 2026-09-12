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
from launch.actions import DeclareLaunchArgument

from launch_pal.include_utils import include_scoped_launch_py_description
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_head_description.launch_arguments import TiagoProHeadArgs

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    camera_model: DeclareLaunchArgument = TiagoProHeadArgs.camera_model
    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    namespace: DeclareLaunchArgument = CommonArgs.namespace
    is_public_sim: DeclareLaunchArgument = CommonArgs.is_public_sim
    sim_type: DeclareLaunchArgument = CommonArgs.sim_type
    mj_control: DeclareLaunchArgument = CommonArgs.mj_control
    world_name: DeclareLaunchArgument = CommonArgs.world_name


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):
    default_controllers = include_scoped_launch_py_description(
        pkg_name='tiago_pro_head_controller_configuration',
        paths=['launch', 'default_controllers.launch.py'],
        launch_arguments={"use_sim_time": launch_args.use_sim_time,
                          "is_public_sim": launch_args.is_public_sim
                          })

    launch_description.add_action(default_controllers)

    play_motion2 = include_scoped_launch_py_description(
        pkg_name='tiago_pro_head_bringup',
        paths=['launch', 'tiago_pro_head_play_motion2.launch.py'],
        launch_arguments={"use_sim_time": launch_args.use_sim_time})

    launch_description.add_action(play_motion2)

    robot_state_publisher = include_scoped_launch_py_description(
        pkg_name='tiago_pro_head_description',
        paths=['launch', 'robot_state_publisher.launch.py'],
        launch_arguments={"camera_model": launch_args.camera_model,
                          "namespace": launch_args.namespace,
                          "use_sim_time": launch_args.use_sim_time,
                          "is_public_sim": launch_args.is_public_sim,
                          "sim_type": launch_args.sim_type,
                          "mj_control": launch_args.mj_control,
                          "world_name": launch_args.world_name,
                          })

    launch_description.add_action(robot_state_publisher)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
