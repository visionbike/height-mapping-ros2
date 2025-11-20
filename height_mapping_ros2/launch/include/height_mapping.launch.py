from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("height_mapping_ros2")

    params_file = os.path.join(pkg_share, "config", "height_mapping_node.yaml")
    vis_params = os.path.join(pkg_share, "config", "include", "heightmap_visualization.yaml")

    debug_arg = DeclareLaunchArgument(
        "debug_mode",
        default_value="false",
        description="Enable debug publishing for height mapping",
    )

    debug_mode = LaunchConfiguration("debug_mode")

    height_mapping_node = Node(
        package="height_mapping_ros2",
        executable="height_mapping_node",
        name="height_mapping_local",
        output="screen",
        parameters=[params_file, {"height_mapping_node.debug_mode": debug_mode}],
    )

    grid_map_visualization = Node(
        package="grid_map_visualization",
        executable="grid_map_visualization",
        name="height_mapping_vis",
        output="log",
        parameters=[vis_params],
    )

    return LaunchDescription(
        [
            debug_arg,
            height_mapping_node,
            grid_map_visualization,
        ]
    )
