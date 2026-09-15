from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    shelf_column_number = LaunchConfiguration(
        "shelf_column_number"
    )

    book_colour = LaunchConfiguration(
        "book_colour"
    )

    image_dir = LaunchConfiguration(
        "image_dir"
    )


    declare_column = DeclareLaunchArgument(
        "shelf_column_number",
        default_value="1",
        description="Target shelf column 1-5."
    )


    declare_colour = DeclareLaunchArgument(
        "book_colour",
        default_value="red",
        description="Target book colour."
    )


    declare_image_dir = DeclareLaunchArgument(
        "image_dir",
        default_value="/erc_images"
    )


    column_detector = Node(
        package="erc_solution",
        executable="shelf_column_detector",
        name="shelf_column_detector",
        output="screen",
        parameters=[
            {
                "shelf_column_number":
                    shelf_column_number,

                "image_dir":
                    image_dir,

                "min_match_score":
                    0.25,

                "confirm_frames":
                    5,
            }
        ],
    )


    navigator = Node(
        package="erc_solution",
        executable="column_navigator",
        name="column_navigator",
        output="screen",
        parameters=[
            {
                # Disable the laser false-positive
                # emergency stop.
                "laser_emergency_stop_m":
                    0.0,

                # Keep the proven camera stop distance.
                "stop_distance_m":
                    2.3,

                "slow_down_distance_m":
                    2.8,

                "approach_linear_speed":
                    0.12,

                "min_linear_speed":
                    0.035,

                # Simulator can run at low RTF.
                "search_timeout_sec":
                    900.0,
            }
        ],
    )


    book_detector = Node(
        package="erc_solution",
        executable="book_colour_detector",
        name="book_colour_detector",
        output="screen",
        parameters=[
            {
                "book_colour":
                    book_colour,

                "image_dir":
                    image_dir,

                # Proven during the manual trial.
                "first_book_row":
                    1,

                # Allows the centre-lock target
                # even when not all four books
                # are simultaneously visible.
                "require_full_column":
                    False,
            }
        ],
    )


    grasp_controller = Node(
        package="erc_solution",
        executable="grasp_controller",
        name="grasp_controller",
        output="screen",
    )


    return LaunchDescription(
        [
            declare_column,
            declare_colour,
            declare_image_dir,
            column_detector,
            navigator,
            book_detector,
            grasp_controller,
        ]
    )
