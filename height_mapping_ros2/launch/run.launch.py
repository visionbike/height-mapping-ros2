from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("height_mapping_ros2")

    global_mapping_arg = DeclareLaunchArgument(
        "global_mapping",
        default_value="false",
        description="Enable global mapping mode instead of local height mapping.",
    )
    debug_mode_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="false",
        description="Enable debug publishing for mapping nodes.",
    )

    global_mapping = LaunchConfiguration("global_mapping")
    debug_mode = LaunchConfiguration("debug_mode")

    height_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "include", "height_mapping.launch.py")
        ),
        condition=UnlessCondition(global_mapping),
        launch_arguments={"debug_mode": debug_mode}.items(),
    )

    global_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "include", "global_mapping.launch.py")
        ),
        condition=IfCondition(global_mapping),
        launch_arguments={"debug_mode": debug_mode}.items(),
    )

    rviz_config_local = PathJoinSubstitution(
        [pkg_share, "launch", "rviz", "height_mapping.rviz"]
    )
    rviz_config_global = PathJoinSubstitution(
        [pkg_share, "launch", "rviz", "global_mapping.rviz"]
    )

    rviz_local = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_height_mapping_local",
        output="log",
        arguments=["-d", rviz_config_local],
        condition=UnlessCondition(global_mapping),
    )

    rviz_global = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_height_mapping_global",
        output="log",
        arguments=["-d", rviz_config_global],
        condition=IfCondition(global_mapping),
    )

    return LaunchDescription(
        [
            global_mapping_arg,
            debug_mode_arg,
            height_mapping_launch,
            global_mapping_launch,
            rviz_local,
            rviz_global,
        ]
    )
