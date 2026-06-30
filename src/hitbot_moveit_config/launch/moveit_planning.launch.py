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
    description_pkg = get_package_share_directory('hitbot_description')
    urdf_file = os.path.join(description_pkg, 'urdf', 'hitbot_real.urdf')
    doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': doc.toxml()}

    moveit_pkg = get_package_share_directory('hitbot_moveit_config')
    srdf_file = os.path.join(moveit_pkg, 'config', 'hitbot.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('hitbot_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    joint_limits_yaml = load_yaml('hitbot_moveit_config', 'config/joint_limits.yaml')

    ompl_yaml = load_yaml('hitbot_moveit_config', 'config/ompl_planning.yaml')
    ompl_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml,
    }

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
        {'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints'},
        {'use_sim_time': False},
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )

    return LaunchDescription([
        move_group_node
    ])
