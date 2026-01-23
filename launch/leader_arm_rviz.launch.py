from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    frame_id = LaunchConfiguration("frame_id")
    leader_joint_names = LaunchConfiguration("leader_joint_names")
    franka_joint_names = LaunchConfiguration("franka_joint_names")

    pkg_share = Path(get_package_share_directory("leader_arm_viz"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("frame_id", default_value="world"),
            DeclareLaunchArgument(
                "leader_joint_names",
                default_value="['joint1','joint2','joint3','joint4','joint5','joint6','joint7_left']",
            ),
            DeclareLaunchArgument(
                "franka_joint_names",
                default_value="['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7']",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="leader_state_publisher",
                parameters=[{"robot_description": _read_text(pkg_share / "urdf" / "cello_description.urdf")}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="franka_state_publisher",
                parameters=[{"robot_description": _read_text(pkg_share / "urdf" / "fr3.urdf")}],
            ),
            Node(
                package="leader_arm_viz",
                executable="leader_joint_publisher",
                name="leader_joint_publisher",
                output='screen',
                parameters=[
                    {
                        "publish_rate_hz": publish_rate_hz,
                        "leader_joint_names": leader_joint_names,
                        "franka_joint_names": franka_joint_names,
                        "frame_id": frame_id,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(use_rviz),
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_leader_frame",
                arguments=["5", "0", "0", "0", "0", "0", "world", "leader_frame"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_fr3_frame",
                arguments=["0", "0", "0", "0", "0", "0", "world", "fr3_frame"],
            ),
        ]
    )
