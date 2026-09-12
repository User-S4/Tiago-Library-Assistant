from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_package'),
        DeclareLaunchArgument('shelf_number'),
        DeclareLaunchArgument('book_colour'),
        LogInfo(msg=['Running: shelf=', LaunchConfiguration('shelf_number'),
                     ' colour=', LaunchConfiguration('book_colour')]),
    ])
