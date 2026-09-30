import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("roa_motion_player")
    player_config = os.path.join(share, "config", "motion_player.yaml")
    robot_description = xacro.process_file(os.path.join(
        share, "description", "urdf", "roa_v4_21dof.xacro")).toxml()
    rviz_config = os.path.join(share, "config", "roa_v4_21dof.rviz")

    return LaunchDescription([
        Node(
            package="roa_motion_player", executable="motion_player",
            parameters=[player_config, {"wait_for_walk_initialized": False}],
            output="screen"),
        Node(
            package="roa_motion_player", executable="hw_cmd_to_joint_state",
            parameters=[{"command_topic": "/hardware_interface/command"}],
            output="screen"),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}], output="screen"),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_config], output="screen"),
    ])
