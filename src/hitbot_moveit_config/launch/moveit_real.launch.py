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
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # ── 1. URDF (use the REAL-hardware variant) ──────────────────────
    description_pkg = get_package_share_directory('hitbot_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'hitbot_real.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    # ── 2. SRDF ──────────────────────────────────────────────────────
    moveit_pkg = get_package_share_directory('hitbot_moveit_config')
    srdf_file = os.path.join(moveit_pkg, 'config', 'hitbot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ── 3. Kinematics ────────────────────────────────────────────────
    kinematics_yaml = load_yaml('hitbot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # ── 4. Joint Limits ──────────────────────────────────────────────
    joint_limits_yaml = load_yaml('hitbot_moveit_config', 'config/joint_limits.yaml')

    # ── 5. OMPL planning pipeline ────────────────────────────────────
    ompl_yaml = load_yaml('hitbot_moveit_config', 'config/ompl_planning.yaml')
    ompl_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml,
    }

    # ── 6. MoveIt controllers ────────────────────────────────────────
    moveit_controllers = load_yaml('hitbot_moveit_config', 'config/moveit_controllers.yaml')
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

    # ── 7. ros2_control controller config ────────────────────────────
    controller_config = os.path.join(
        description_pkg, 'config', 'hitbot_controllers.yaml')

    # ── Nodes ────────────────────────────────────────────────────────

    # ros2_control controller manager (loads the hardware interface)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Joint state broadcaster (publishes /joint_states from hardware)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Joint trajectory controller (receives trajectory from MoveIt)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Robot state publisher (publishes TF from URDF + /joint_states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
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
        {'use_sim_time': False},
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    # RViz
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
