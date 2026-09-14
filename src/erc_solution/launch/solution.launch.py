"""
Competition entry point for ERC 2026 Phase 1.

Run after the simulation is already up:

  ros2 launch erc_bringup simulation.launch.py
  ros2 launch erc_solution solution.launch.py shelf_column_number:=2 book_colour:=red

The two arguments say which column and colour to target. They do not say where
those are: marker numbers and book positions are randomised on every load, so
everything still has to be found visually.

Three nodes, one responsibility each:

  shelf_column_detector  reads the number plaques, publishes the identified
                         column and a steering offset towards it
  column_navigator       consumes that offset to search, centre, approach and
                         settle in front of the target column
  book_colour_detector   reads the four books in the centred column and
                         publishes the target colour's shelf row

They communicate only over topics, so any one of them can be restarted, replaced
or run alone during development without disturbing the others.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    shelf_column_number = LaunchConfiguration("shelf_column_number")
    book_colour = LaunchConfiguration("book_colour")
    image_dir = LaunchConfiguration("image_dir")

    declare_column = DeclareLaunchArgument(
        "shelf_column_number",
        default_value="1",
        description="Target shelf column, 1-5. Its position is found visually.",
    )
    declare_colour = DeclareLaunchArgument(
        "book_colour",
        default_value="red",
        description="Target book colour: red, blue, green or yellow.",
    )
    declare_image_dir = DeclareLaunchArgument(
        "image_dir",
        default_value="erc_images",
        description="Where annotated identification images are written.",
    )

    column_detector = Node(
        package="erc_solution",
        executable="shelf_column_detector",
        name="shelf_column_detector",
        output="screen",
        parameters=[{
            "shelf_column_number": shelf_column_number,
            "image_dir": image_dir,
        }],
    )

    navigator = Node(
        package="erc_solution",
        executable="column_navigator",
        name="column_navigator",
        output="screen",
    )

    book_detector = Node(
        package="erc_solution",
        executable="book_colour_detector",
        name="book_colour_detector",
        output="screen",
        parameters=[{
            "book_colour": book_colour,
            "image_dir": image_dir,
        }],
    )

    return LaunchDescription([
        declare_column,
        declare_colour,
        declare_image_dir,
        column_detector,
        navigator,
        book_detector,
    ])
