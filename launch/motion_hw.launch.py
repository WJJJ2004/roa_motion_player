import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("roa_motion_player")
    player_config = os.path.join(share, "config", "motion_player.yaml")
    return LaunchDescription([
        Node(
            package="roa_motion_player", executable="motion_player",
            parameters=[player_config, {"wait_for_walk_initialized": True}],
            output="screen"),
    ])
