from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("height_mapping_ros2")

    params_file = os.path.join(pkg_share, "config", "global_mapping_node.yaml")

    debug_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="false",
        description="Enable debug publishing for global mapping",
    )
    debug_mode = LaunchConfiguration("debug_mode")

    global_mapping_node = Node(
        package="height_mapping_ros2",
        executable="global_mapping_node",
        name="height_mapping_global",
        output="screen",
        parameters=[params_file, {"global_mapping_node.debug_mode": debug_mode}],
    )

    return LaunchDescription([debug_arg, global_mapping_node])
