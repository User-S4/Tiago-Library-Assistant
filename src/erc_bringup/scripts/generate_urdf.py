#!/usr/bin/env python3
"""
Generate the competition URDF for the TIAGo Pro robot.

Calls PAL's xacro with the specified configuration, then applies all Gazebo
Harmonic patches (MecanumDrive plugin, wheel friction, camera format fix,
head camera → Intel RealSense D435 retarget + optical-frame fix,
controller config swap). The result is written to
erc_description/urdf/tiago_pro.urdf for use by the simulation launch file.

Run inside the container after building:
    python3 /opt/erc_ws/src/erc_bringup/scripts/generate_urdf.py

Use --help to see all available configuration options and their valid values.

Examples:
    # Default competition config (dual-arm, pro grippers)
    python3 src/erc_bringup/scripts/generate_urdf.py

    # Single left arm only
    python3 src/erc_bringup/scripts/generate_urdf.py --arm_type_right no-arm

    # With wrist camera
    python3 src/erc_bringup/scripts/generate_urdf.py --has_wrist_camera

    # No arms (navigation-only testing)
    python3 src/erc_bringup/scripts/generate_urdf.py --arm_type_left no-arm --arm_type_right no-arm
"""

import argparse
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory


def generate_urdf(**xacro_args):
    """Generate and patch the TIAGo Pro URDF. Returns the URDF string."""

    tiago_dir = get_package_share_directory('tiago_pro_description')
    bringup_dir = get_package_share_directory('erc_bringup')

    controller_cfg = os.path.join(
        bringup_dir, 'config', 'gazebo_controller_manager_cfg.yaml')

    # ── Generate URDF via xacro ──
    xacro_path = os.path.join(tiago_dir, 'robots', 'tiago_pro.urdf.xacro')
    cmd = ['xacro', xacro_path]
    for key, val in xacro_args.items():
        cmd.append(f'{key}:={val}')
    urdf = subprocess.check_output(cmd).decode('utf-8')

    # ── Swap PAL's controller config with ours ──
    pal_cfg = os.path.join(
        tiago_dir, 'ros2_control', 'gazebo_controller_manager_cfg.yaml')
    urdf = urdf.replace(pal_cfg, controller_cfg)

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 1: Inject MecanumDrive plugin
    # ══════════════════════════════════════════════════════════════════════
    # The gz-sim MecanumDrive system plugin drives the omni base. It's
    # injected into the existing gz_ros2_control <gazebo> block so both
    # plugins share one SDF <plugin> parent.
    mecanum_plugin = (
        '    <plugin filename="gz-sim-mecanum-drive-system"\n'
        '            name="gz::sim::systems::MecanumDrive">\n'
        '      <front_left_joint>wheel_front_left_joint</front_left_joint>\n'
        '      <front_right_joint>wheel_front_right_joint</front_right_joint>\n'
        '      <back_left_joint>wheel_rear_left_joint</back_left_joint>\n'
        '      <back_right_joint>wheel_rear_right_joint</back_right_joint>\n'
        '      <wheel_separation>0.44715</wheel_separation>\n'
        '      <wheelbase>0.488</wheelbase>\n'
        '      <wheel_radius>0.0762</wheel_radius>\n'
        '      <topic>cmd_vel</topic>\n'
        '      <odom_topic>odom</odom_topic>\n'
        '      <odom_publish_frequency>50</odom_publish_frequency>\n'
        '      <frame_id>odom</frame_id>\n'
        '      <child_frame_id>base_footprint</child_frame_id>\n'
        '      <tf_topic>odom_tf</tf_topic>\n'
        '      <min_velocity>-1.0</min_velocity>\n'
        '      <max_velocity>1.0</max_velocity>\n'
        '      <min_acceleration>-1.0</min_acceleration>\n'
        '      <max_acceleration>1.0</max_acceleration>\n'
        '    </plugin>\n'
    )
    urdf = urdf.replace(
        '</plugin>\n  </gazebo>',
        '</plugin>\n' + mecanum_plugin + '  </gazebo>',
        1,
    )

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 2: Strip wheel joints from <ros2_control>
    # ══════════════════════════════════════════════════════════════════════
    # Only MecanumDrive should drive the wheels. Remove them from the
    # ros2_control block so gz_ros2_control doesn't fight over them.
    for _wheel in ('wheel_front_right_joint', 'wheel_front_left_joint',
                   'wheel_rear_right_joint', 'wheel_rear_left_joint'):
        _pos = {'wheel_front_right_joint': 'front_right',
                'wheel_front_left_joint':  'front_left',
                'wheel_rear_right_joint':  'rear_right',
                'wheel_rear_left_joint':   'rear_left'}[_wheel]
        _joint_block = (
            f'<joint name="{_wheel}">\n'
            f'      <command_interface name="velocity"/>\n'
            f'      <state_interface name="position"/>\n'
            f'    </joint>\n'
            f'    <transmission name="wheel_{_pos}_trans">\n'
            f'      <plugin>transmission_interface/SimpleTransmission</plugin>\n'
            f'      <actuator name="wheel_{_pos}_actuator" role="actuator1"/>\n'
            f'      <joint name="{_wheel}" role="joint1">\n'
            f'        <mechanical_reduction>1.0</mechanical_reduction>\n'
            f'      </joint>\n'
            f'    </transmission>\n'
        )
        if _joint_block in urdf:
            urdf = urdf.replace(_joint_block, '', 1)
        else:
            print(f'WARNING: could not strip {_wheel} from <ros2_control> — '
                  f'block format may have changed.')

    # Re-inject wheel joints as state-only (no command interface, no
    # transmission) so joint_state_broadcaster can publish their positions
    # for RViz. MecanumDrive still commands them — no conflict.
    _state_only_wheels = ''
    for _wheel in ('wheel_front_right_joint', 'wheel_front_left_joint',
                   'wheel_rear_right_joint', 'wheel_rear_left_joint'):
        _state_only_wheels += (
            f'    <joint name="{_wheel}">\n'
            f'      <state_interface name="position"/>\n'
            f'      <state_interface name="velocity"/>\n'
            f'    </joint>\n'
        )
    urdf = urdf.replace('</ros2_control>', _state_only_wheels + '  </ros2_control>', 1)

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 3: Zero lateral wheel friction for mecanum strafe
    # ══════════════════════════════════════════════════════════════════════
    _n = urdf.count('<mu2>0.30</mu2>')
    urdf = urdf.replace('<mu2>0.30</mu2>', '<mu2>0.0</mu2>')
    if _n != 4:
        print(f'WARNING: expected 4 wheel mu2=0.30 blocks, found {_n} — '
              f'strafe may not work.')

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 4: Fix camera pixel format (B8G8R8 → R8G8B8)
    # ══════════════════════════════════════════════════════════════════════
    _nc = urdf.count('<format>B8G8R8</format>')
    urdf = urdf.replace('<format>B8G8R8</format>', '<format>R8G8B8</format>')
    if _nc == 0:
        print('WARNING: no B8G8R8 camera format found — '
              'camera format fix did not apply.')

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 4b: Fix head camera header frame to the OPTICAL frame
    # ══════════════════════════════════════════════════════════════════════
    # PAL points the head camera sensors' <gz_frame_id> at the camera BODY
    # frames (head_front_camera_{color,depth}_frame — X forward, Z up). But the
    # rendered image/depth/camera_info follow the ROS optical convention (Z
    # forward, X right, Y down). Publishing them stamped with the body frame
    # breaks every consumer that trusts the header: RViz's Camera/DepthCloud
    # projects through the wrong axes, and a back-projected point "1 m in front"
    # gets placed "1 m up" (robot_perception's depth_to_cloud stamps its cloud
    # with exactly this frame). Point the sensors at the matching *_optical_frame
    # (already present in the URDF) so headers are correct at the source.
    for _body, _opt in (
        ('<gz_frame_id>head_front_camera_color_frame</gz_frame_id>',
         '<gz_frame_id>head_front_camera_color_optical_frame</gz_frame_id>'),
        ('<gz_frame_id>head_front_camera_depth_frame</gz_frame_id>',
         '<gz_frame_id>head_front_camera_depth_optical_frame</gz_frame_id>'),
    ):
        if _body in urdf:
            urdf = urdf.replace(_body, _opt, 1)
        else:
            print(f'WARNING: head camera gz_frame_id not found ({_body}); '
                  f'RViz camera projection may be mis-framed.')

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 4c: Match the real Intel RealSense D435 depth module (head camera)
    # ══════════════════════════════════════════════════════════════════════
    # PAL renders the head camera at 640x480 with an 85.2° H-FoV and a short
    # 0.3–3.0 m depth clip — unlike a real D435. Retarget BOTH the colour and the
    # depth head sensors to a D435 depth mode: 640x360 (a native 16:9 D435 depth
    # resolution) at an 87° H-FoV (≈87°x56°, close to the datasheet 87°x58°) and
    # open the depth range to 0.2–8 m so the shelf at ~4 m is in range. Colour is
    # kept matched to depth (same resolution + FoV) so the two stay pixel-aligned
    # — depth_to_cloud's colouring and the detector's depth sampling both rely on
    # that 1:1 correspondence. Sensor-realistic depth noise / disparity
    # quantisation / edge holes are added downstream in robot_perception's
    # depth_to_cloud node. NOTE: this runs BEFORE PATCH 5 so the global replaces
    # touch ONLY the head camera, never the (later-injected) wrist cameras.
    _cam = {
        '<horizontal_fov>1.4870205226991688</horizontal_fov>':
            '<horizontal_fov>1.5184364492350666</horizontal_fov>',   # 87 deg
        '<vertical_fov>1.0122909661567112</vertical_fov>':
            '<vertical_fov>0.9773843811168246</vertical_fov>',        # ~56 deg
        '<height>480</height>': '<height>360</height>',
        '<near>0.3</near>': '<near>0.2</near>',
        '<far>3.0</far>': '<far>8.0</far>',
    }
    for _old, _new in _cam.items():
        if _old not in urdf:
            print(f'WARNING: head camera spec token not found ({_old}); '
                  f'D435 match incomplete.')
        urdf = urdf.replace(_old, _new)

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 5: Inject wrist camera Gazebo sensor plugins + frames
    # ══════════════════════════════════════════════════════════════════════
    # PAL's wrist camera xacro (pal_sea_arm_description) only adds the
    # physical camera link/joint — no Gazebo sensor plugin or optical
    # frames. We inject the RealSense frame chain (depth, color, optical
    # frames) and sensor blocks modeled after the head camera.
    for side in ('left', 'right'):
        camera_link = f'gripper_{side}_camera_link'
        if f'name="{camera_link}"' not in urdf:
            continue

        prefix = f'gripper_{side}_camera'

        # Frame chain: camera_link → depth_frame → depth_optical_frame
        #              camera_link → color_frame → color_optical_frame
        frames = (
            f'  <link name="{prefix}_depth_frame"/>\n'
            f'  <joint name="{prefix}_depth_joint" type="fixed">\n'
            f'    <origin rpy="0 0 0" xyz="0 0 0"/>\n'
            f'    <parent link="{camera_link}"/>\n'
            f'    <child link="{prefix}_depth_frame"/>\n'
            f'  </joint>\n'
            f'  <link name="{prefix}_depth_optical_frame"/>\n'
            f'  <joint name="{prefix}_depth_optical_joint" type="fixed">\n'
            f'    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>\n'
            f'    <parent link="{prefix}_depth_frame"/>\n'
            f'    <child link="{prefix}_depth_optical_frame"/>\n'
            f'  </joint>\n'
            f'  <link name="{prefix}_color_frame"/>\n'
            f'  <joint name="{prefix}_color_joint" type="fixed">\n'
            f'    <origin rpy="0 0 0" xyz="0 0.015 0"/>\n'
            f'    <parent link="{camera_link}"/>\n'
            f'    <child link="{prefix}_color_frame"/>\n'
            f'  </joint>\n'
            f'  <link name="{prefix}_color_optical_frame"/>\n'
            f'  <joint name="{prefix}_color_optical_joint" type="fixed">\n'
            f'    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>\n'
            f'    <parent link="{prefix}_color_frame"/>\n'
            f'    <child link="{prefix}_color_optical_frame"/>\n'
            f'  </joint>\n'
        )

        sensors = (
            f'  <gazebo reference="{camera_link}">\n'
            f'    <sensor name="{prefix}" type="camera">\n'
            f'      <topic>{prefix}/color</topic>\n'
            f'      <gz_frame_id>{camera_link}</gz_frame_id>\n'
            f'      <always_on>true</always_on>\n'
            f'      <update_rate>30</update_rate>\n'
            f'      <visualize>false</visualize>\n'
            f'      <camera name="{prefix}">\n'
            f'        <horizontal_fov>1.4870205226991688</horizontal_fov>\n'
            f'        <image>\n'
            f'          <format>R8G8B8</format>\n'
            f'          <width>640</width>\n'
            f'          <height>480</height>\n'
            f'        </image>\n'
            f'      </camera>\n'
            f'    </sensor>\n'
            f'    <sensor name="{prefix}_depth_sensor" type="rgbd_camera">\n'
            f'      <always_on>true</always_on>\n'
            f'      <gz_frame_id>{camera_link}</gz_frame_id>\n'
            f'      <update_rate>30</update_rate>\n'
            f'      <visualize>false</visualize>\n'
            f'      <topic>{prefix}/depth</topic>\n'
            f'      <camera name="{prefix}_rgbd">\n'
            f'        <horizontal_fov>1.4870205226991688</horizontal_fov>\n'
            f'        <image>\n'
            f'          <width>640</width>\n'
            f'          <height>480</height>\n'
            f'          <format>R8G8B8</format>\n'
            f'        </image>\n'
            f'        <depth_camera>\n'
            f'          <clip>\n'
            f'            <near>0.1</near>\n'
            f'            <far>2.0</far>\n'
            f'          </clip>\n'
            f'        </depth_camera>\n'
            f'      </camera>\n'
            f'    </sensor>\n'
            f'  </gazebo>\n'
        )

        urdf = urdf.replace('</robot>', frames + sensors + '</robot>')
        print(f'Injected wrist camera frames and sensors for {camera_link}')

    # ══════════════════════════════════════════════════════════════════════
    #  PATCH 6: Inject contact sensors on all robot links
    # ══════════════════════════════════════════════════════════════════════
    # Adds Gazebo contact sensors on all collision-bearing links, grouped
    # into logical ROS topics. Requires gz-sim-contact-system in the world
    # SDF. Multiple sensors publish to the same ROS topic via the bridge.
    # Format: (urdf_link, ros_topic, sdf_collision_name)
    contact_sensors = [
        # ── Base ──
        ('base_footprint', 'contacts/base', 'base_footprint_fixed_joint_lump__base_link_collision'),
        # ── Torso ──
        ('torso_lift_link', 'contacts/torso', 'torso_lift_link_collision'),
        # ── Head ──
        ('head_1_link', 'contacts/head', 'head_1_link_collision'),
        ('head_2_link', 'contacts/head', 'head_2_link_collision'),
        # ── Left arm ──
        ('arm_left_1_link', 'contacts/arm_left', 'arm_left_1_link_collision'),
        ('arm_left_2_link', 'contacts/arm_left', 'arm_left_2_link_collision'),
        ('arm_left_3_link', 'contacts/arm_left', 'arm_left_3_link_collision'),
        ('arm_left_4_link', 'contacts/arm_left', 'arm_left_4_link_collision'),
        ('arm_left_5_link', 'contacts/arm_left', 'arm_left_5_link_collision'),
        ('arm_left_6_link', 'contacts/arm_left', 'arm_left_6_link_collision'),
        ('arm_left_7_link', 'contacts/arm_left', 'arm_left_7_link_collision'),
        # ── Right arm ──
        ('arm_right_1_link', 'contacts/arm_right', 'arm_right_1_link_collision'),
        ('arm_right_2_link', 'contacts/arm_right', 'arm_right_2_link_collision'),
        ('arm_right_3_link', 'contacts/arm_right', 'arm_right_3_link_collision'),
        ('arm_right_4_link', 'contacts/arm_right', 'arm_right_4_link_collision'),
        ('arm_right_5_link', 'contacts/arm_right', 'arm_right_5_link_collision'),
        ('arm_right_6_link', 'contacts/arm_right', 'arm_right_6_link_collision'),
        ('arm_right_7_link', 'contacts/arm_right', 'arm_right_7_link_collision'),
        # ── Left gripper ──
        ('gripper_left_fingertip_left_link', 'contacts/gripper_left', 'gripper_left_fingertip_left_link_collision'),
        ('gripper_left_fingertip_right_link', 'contacts/gripper_left', 'gripper_left_fingertip_right_link_collision'),
        ('gripper_left_inner_finger_left_link', 'contacts/gripper_left', 'gripper_left_inner_finger_left_link_collision'),
        ('gripper_left_inner_finger_right_link', 'contacts/gripper_left', 'gripper_left_inner_finger_right_link_collision'),
        ('gripper_left_outer_finger_left_link', 'contacts/gripper_left', 'gripper_left_outer_finger_left_link_collision'),
        ('gripper_left_outer_finger_right_link', 'contacts/gripper_left', 'gripper_left_outer_finger_right_link_collision'),
        # ── Right gripper ──
        ('gripper_right_fingertip_left_link', 'contacts/gripper_right', 'gripper_right_fingertip_left_link_collision'),
        ('gripper_right_fingertip_right_link', 'contacts/gripper_right', 'gripper_right_fingertip_right_link_collision'),
        ('gripper_right_inner_finger_left_link', 'contacts/gripper_right', 'gripper_right_inner_finger_left_link_collision'),
        ('gripper_right_inner_finger_right_link', 'contacts/gripper_right', 'gripper_right_inner_finger_right_link_collision'),
        ('gripper_right_outer_finger_left_link', 'contacts/gripper_right', 'gripper_right_outer_finger_left_link_collision'),
        ('gripper_right_outer_finger_right_link', 'contacts/gripper_right', 'gripper_right_outer_finger_right_link_collision'),
    ]
    _injected = 0
    for link_name, topic, collision_name in contact_sensors:
        if f'name="{link_name}"' not in urdf:
            continue
        contact_block = (
            f'  <gazebo reference="{link_name}">\n'
            f'    <sensor name="{link_name}_contact" type="contact">\n'
            f'      <always_on>true</always_on>\n'
            f'      <update_rate>30</update_rate>\n'
            f'      <topic>contacts/{link_name}</topic>\n'
            f'      <contact>\n'
            f'        <collision>{collision_name}</collision>\n'
            f'      </contact>\n'
            f'    </sensor>\n'
            f'  </gazebo>\n'
        )
        urdf = urdf.replace('</robot>', contact_block + '</robot>')
        _injected += 1
    print(f'Injected {_injected} contact sensors across base/torso/head/arms/grippers')

    return urdf


def main():
    parser = argparse.ArgumentParser(
        description='Generate the ERC 2026 TIAGo Pro URDF with Gazebo Harmonic patches.')

    # ── Robot configuration ──
    parser.add_argument('--arm_type_left', default='tiago-pro-s',
                        choices=['tiago-pro-s', 'no-arm'],
                        help='Left arm type (default: tiago-pro-s)')
    parser.add_argument('--arm_type_right', default='tiago-pro-s',
                        choices=['tiago-pro-s', 'no-arm'],
                        help='Right arm type (default: tiago-pro)')
    parser.add_argument('--end_effector_left', default='pal-pro-gripper',
                        choices=['pal-pro-gripper', 'no-end-effector'],
                        help='Left end effector (default: pal-pro-gripper)')
    parser.add_argument('--end_effector_right', default='pal-pro-gripper',
                        choices=['pal-pro-gripper', 'no-end-effector'],
                        help='Right end effector (default: pal-pro-gripper)')
    parser.add_argument('--ft_sensor_left', default=None,
                        choices=['no-ft-sensor'],
                        help='Force torque sensor left arm [no-ft-sensor]')
    parser.add_argument('--ft_sensor_right', default=None,
                        choices=['no-ft-sensor'],
                        help='Force torque sensor right arm [no-ft-sensor]')
    parser.add_argument('--wrist_model_left', default=None,
                        help='Wrist model left arm [wrist-2010, wrist-2017]')
    parser.add_argument('--wrist_model_right', default=None,
                        help='Wrist model right arm [wrist-2010, wrist-2017]')
    parser.add_argument('--camera_model', default=None,
                        help='Head camera model [realsense-d435i, realsense-d435]')
    parser.add_argument('--laser_model', default=None,
                        help='LiDAR model [sick-571, sick-561, sick-tim, no-laser]')
    parser.add_argument('--base_type', default=None,
                        help='Base type [omni_base]')

    # ── Boolean flags ──
    parser.add_argument('--has_wrist_camera', action='store_true',
                        help='Mount a wrist camera on the end effector [flag, default: off]')

    args = parser.parse_args()

    # Build the xacro argument dict — always include these
    xacro_args = {
        'arm_type_left': args.arm_type_left,
        'arm_type_right': args.arm_type_right,
        'end_effector_left': args.end_effector_left,
        'end_effector_right': args.end_effector_right,
        'has_teleop_arms': 'False',
        'is_public_sim': 'True',
        'use_sim_time': 'True',
        'gazebo_version': 'gazebo',
    }

    # Only pass optional args if explicitly set (let xacro use its own defaults)
    optional = {
        'ft_sensor_left': args.ft_sensor_left,
        'ft_sensor_right': args.ft_sensor_right,
        'wrist_model_left': args.wrist_model_left,
        'wrist_model_right': args.wrist_model_right,
        'camera_model': args.camera_model,
        'laser_model': args.laser_model,
        'base_type': args.base_type,
    }
    for key, val in optional.items():
        if val is not None:
            xacro_args[key] = val

    if args.has_wrist_camera:
        xacro_args['has_wrist_camera'] = 'True'

    print('[generate_urdf] Configuration:')
    for key, val in xacro_args.items():
        print(f'  {key}: {val}')

    urdf = generate_urdf(**xacro_args)

    # Write to erc_description/urdf/ (in the source tree, visible on host)
    output_dir = os.path.join(
        os.path.dirname(__file__), '..', '..', 'erc_description', 'urdf')
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, 'tiago_pro.urdf')

    with open(output_path, 'w') as f:
        f.write(urdf)

    print(f'[generate_urdf] Written to {os.path.realpath(output_path)}')
    print(f'[generate_urdf] URDF size: {len(urdf):,} bytes')


if __name__ == '__main__':
    main()
