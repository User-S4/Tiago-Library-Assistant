"""Rebuild the ERC head depth-camera point cloud with a RealSense-D435 noise model.

The Gazebo ``rgbd_camera`` advertises ``/head_front_camera/depth/points`` but
never fills it. This node back-projects the float32 depth image through
``camera_info``, applies a RealSense-D435-like noise model, and publishes a
normal ``sensor_msgs/PointCloud2`` (plus a field-of-view frustum marker).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    color_source = LaunchConfiguration('color_source')
    realsense_noise = LaunchConfiguration('realsense_noise')

    return LaunchDescription([
        DeclareLaunchArgument(
            'color_source', default_value='depth',
            description="Cloud colouring: 'depth' (turbo by distance, "
                        "RealSense-Viewer look), 'rgb' (fused colour image), "
                        "or 'none' (plain XYZ)"),
        DeclareLaunchArgument(
            'realsense_noise', default_value='true',
            description='Apply the RealSense-D435 depth noise model'),
        Node(
            package='sensors',
            executable='depth_to_cloud',
            name='depth_to_cloud',
            output='screen',
            parameters=[{
                'color_source': color_source,
                'realsense_noise': realsense_noise,
                'use_sim_time': True,
            }],
        ),
    ])
