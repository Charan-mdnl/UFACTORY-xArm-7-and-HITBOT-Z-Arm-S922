import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ── 1. URDF ──────────────────────
    description_pkg = get_package_share_directory('hitbot_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'hitbot_real.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # ── 2. SRDF ──────────────────────────────────────────────────────
    moveit_pkg = get_package_share_directory('hitbot_moveit_config')
    srdf_file = os.path.join(moveit_pkg, 'config', 'hitbot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    test_node = Node(
        package="hitbot_moveit_config",
        executable="test_moveit_sequence",
        name="test_moveit_sequence",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return LaunchDescription([test_node])
