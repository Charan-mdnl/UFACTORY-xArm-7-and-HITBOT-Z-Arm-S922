"""
Dual-Arm Simulation Launch (fake hardware / mock_components)

Launches both xArm 7 and HITBOT Z-Arm S922 in simulation mode using
mock_components/GenericSystem so no real robot hardware is needed.

Usage:
  ros2 launch dual_arm_config dual_arm_sim.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import xacro


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    pkg = get_package_share_directory('dual_arm_config')

    # ── 1. URDF (xacro → XML) ──────────────────────────────────────
    urdf_file = os.path.join(pkg, 'urdf', 'dual_arm.urdf.xacro')
    doc = xacro.process_file(urdf_file, mappings={'hw_mode': 'sim'})
    robot_description = {'robot_description': doc.toxml()}

    # ── 2. SRDF ─────────────────────────────────────────────────────
    srdf_file = os.path.join(pkg, 'config', 'dual_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ── 3. Kinematics ───────────────────────────────────────────────
    kinematics_yaml = load_yaml('dual_arm_config', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    # ── 4. Joint Limits ─────────────────────────────────────────────
    joint_limits_yaml = load_yaml('dual_arm_config', 'config/joint_limits.yaml')

    # ── 5. OMPL planning ────────────────────────────────────────────
    ompl_yaml = load_yaml('dual_arm_config', 'config/ompl_planning.yaml')
    ompl_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml,
    }

    # ── 6. MoveIt controllers ───────────────────────────────────────
    moveit_controllers = load_yaml(
        'dual_arm_config', 'config/moveit_controllers.yaml')
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ── 7. ros2_control controller config ───────────────────────────
    controller_config = os.path.join(
        pkg, 'config', 'dual_arm_controllers.yaml')

    # ════════════════════════════════════════════════════════════════
    # NODES
    # ════════════════════════════════════════════════════════════════

    # ros2_control_node (loads mock hardware for both arms)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # xArm trajectory controller
    xarm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # HITBOT trajectory controller
    hitbot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hitbot_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # MoveIt move_group
    move_group_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits_yaml,
        ompl_config,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor,
        {'planning_plugin': 'ompl_interface/OMPLPlanner'},
        {'request_adapters': 'default_planning_request_adapters/ResolveConstraintFrames default_planning_request_adapters/ValidateWorkspaceBounds default_planning_request_adapters/CheckStartStateBounds default_planning_request_adapters/CheckStartStateCollision'},
        {'use_sim_time': False},
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # RViz
    rviz_config = os.path.join(pkg, 'rviz', 'dual_arm.rviz')
    rviz_params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        {'use_sim_time': False},
    ]
    # Only add rviz config if it exists
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=rviz_params,
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        xarm_controller_spawner,
        hitbot_controller_spawner,
        move_group_node,
        rviz_node,
    ])
