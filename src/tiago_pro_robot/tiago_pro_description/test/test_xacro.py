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

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from tiago_pro_description.launch_arguments import TiagoProArgs

from urdf_test.xacro_test import define_xacro_test

xacro_file_path = Path(
    get_package_share_directory('tiago_pro_description'),
    'robots',
    'tiago_pro.urdf.xacro',
)

arm_args = (
    TiagoProArgs.arm_type_right,
    TiagoProArgs.arm_type_left,
)
wrist_args_left = (
    TiagoProArgs.wrist_model_left,
    TiagoProArgs.tool_changer_left,
    TiagoProArgs.ft_sensor_left,
    TiagoProArgs.has_wrist_camera
)
wrist_args_right = (
    TiagoProArgs.wrist_model_right,
    TiagoProArgs.tool_changer_right,
    TiagoProArgs.ft_sensor_right,
    TiagoProArgs.has_wrist_camera
)


def exclude_allegro_hand(end_effector):
    _choices = getattr(end_effector, 'choices', None)
    _name = getattr(end_effector, 'name', None)
    filtered_choices = [c for c in _choices if 'allegro-hand' not in str(c)]
    end_effector = DeclareLaunchArgument(name=_name, choices=filtered_choices)
    return end_effector


def force_arg_value(arg_obj, value):
    return DeclareLaunchArgument(name=arg_obj.name, default_value=str(value), choices=[str(value)])


if not os.environ.get('PAL_DISTRO'):
    end_effector_left = exclude_allegro_hand(TiagoProArgs.end_effector_left)
    end_effector_right = exclude_allegro_hand(TiagoProArgs.end_effector_right)
    gripper_args = (end_effector_left, end_effector_right)
else:
    end_effector_left = TiagoProArgs.end_effector_left
    end_effector_right = TiagoProArgs.end_effector_right
    gripper_args = (end_effector_left, end_effector_right)

teleop_args = (
    TiagoProArgs.ft_sensor_teleop_right,
    TiagoProArgs.ft_sensor_teleop_left,
)
test_xacro_base = define_xacro_test(
    xacro_file_path, arm_args, TiagoProArgs.base_type)
test_xacro_laser = define_xacro_test(
    xacro_file_path, arm_args, TiagoProArgs.laser_model)
test_xacro_ee = define_xacro_test(xacro_file_path, arm_args, gripper_args)
test_xacro_ee = define_xacro_test(
    xacro_file_path, TiagoProArgs.arm_type_left, wrist_args_left)
test_xacro_ee = define_xacro_test(
    xacro_file_path, TiagoProArgs.arm_type_right, wrist_args_right)
test_xacro_ee = define_xacro_test(
    xacro_file_path, end_effector_left, wrist_args_left)
test_xacro_ee = define_xacro_test(
    xacro_file_path, end_effector_right, wrist_args_right)
test_xacro_teleop_active = define_xacro_test(
    xacro_file_path, force_arg_value(TiagoProArgs.has_teleop_arms, 'True'), teleop_args)
test_xacro_wrist_camera = define_xacro_test(
    xacro_file_path, force_arg_value(TiagoProArgs.has_wrist_camera, 'True'),
    wrist_args_left + wrist_args_right)
