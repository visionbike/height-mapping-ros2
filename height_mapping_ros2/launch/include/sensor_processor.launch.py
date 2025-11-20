from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("height_mapping_ros2")
    params_file = os.path.join(pkg_share, "config", "sensor_processor_node.yaml")

    sensor_processor = Node(
        package="height_mapping_ros2",
        executable="sensor_processor_node",
        name="sensor_processor",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([sensor_processor])
