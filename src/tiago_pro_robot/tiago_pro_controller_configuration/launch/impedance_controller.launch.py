# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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
import os
from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description
from launch.actions import GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchContext

from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from launch_pal.param_utils import parse_parametric_yaml
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    side: DeclareLaunchArgument = DeclareLaunchArgument(
        name="side",
        default_value="right",
        choices=["right", "left"],
        description="Side of the robot to test.",
    )
    chained_mode: DeclareLaunchArgument = DeclareLaunchArgument(
        name="chained_mode",
        default_value="false",
        choices=["true", "false"],
        description="If true, uses the chained controller config YAML.",
    )


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(
        function=setup_controller_configuration))

    pal_impedance_control = GroupAction([generate_load_controller_launch_description(
        controller_name=LaunchConfiguration("controller_name"),
        controller_params_file=LaunchConfiguration("controller_config"),
        extra_spawner_args=["--inactive"])])

    launch_description.add_action(pal_impedance_control)

    arm_pos_control = GroupAction([generate_load_controller_launch_description(
        controller_name=LaunchConfiguration("pos_controller_name"),
        controller_params_file=LaunchConfiguration("controller_config"),
        extra_spawner_args=["--inactive"])])

    launch_description.add_action(arm_pos_control)
    return


def setup_controller_configuration(context: LaunchContext):
    side = read_launch_argument("side", context)
    if side not in ["left", "right"]:
        raise ValueError(f"Invalid side '{side}'. Must be 'left' or 'right'.")

    controller_name = f"pal_impedance_control_{side}"

    chained_mode_str = read_launch_argument("chained_mode", context)
    if chained_mode_str not in ["true", "false"]:
        raise ValueError(
            f"Invalid chained_mode '{chained_mode_str}'. Must be 'true' or 'false'."
        )
    chained_mode = (chained_mode_str == "true")

    param_file_unchained = os.path.join(
        get_package_share_directory("tiago_pro_controller_configuration"),
        "config",
        "impedance_controller_arm.yaml",
    )

    param_file_chained = os.path.join(
        get_package_share_directory("tiago_pro_controller_configuration"),
        "config",
        "impedance_controller_arm_jtc.yaml",
    )

    param_file = param_file_chained if chained_mode else param_file_unchained

    if not os.path.exists(param_file):
        raise FileNotFoundError(f"Parameter file not found: {param_file}")

    # Populate placeholders like ${ARM_SIDE_PREFIX} in your YAML.
    remappings = {"ARM_SIDE_PREFIX": side}

    parsed_yaml = parse_parametric_yaml(
        source_files=[param_file],
        param_rewrites=remappings,
    )

    return [
        SetLaunchConfiguration("controller_name", controller_name),
        SetLaunchConfiguration("pos_controller_name", f"arm_{side}_pos_controller"),
        SetLaunchConfiguration("controller_config", parsed_yaml),
    ]


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
