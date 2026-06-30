"""Bring up the HITBOT in Gazebo + MoveIt for imitation playback.

Starts (all with use_sim_time):
  * robot_state_publisher  (hitbot.urdf — the Gazebo twin, same kinematics as
    the model move_group plans with)
  * Gazebo (headless) + spawn the robot + load joint_state_broadcaster and
    arm_controller
  * move_group  (provides /compute_ik and trajectory execution)
  * RViz        (optional: gui:=true)

Then, in a second terminal, run the executor on a retargeted trajectory::

    ros2 run hitbot_imitation execute_moveit --ros-args \
        -p trajectory:=$PWD/ee_traj.npz -p dry_run:=false

Usage::

    ros2 launch hitbot_imitation imitation.launch.py
    ros2 launch hitbot_imitation imitation.launch.py gui:=true
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    description_pkg = get_package_share_directory("hitbot_description")
    moveit_pkg = get_package_share_directory("hitbot_moveit_config")

    gui = LaunchConfiguration("gui")
    use_sim_time = {"use_sim_time": True}

    # --- robot description: the Gazebo twin (hitbot.urdf) ----------------- #
    sim_urdf = os.path.join(description_pkg, "urdf", "hitbot.urdf")
    robot_description = {"robot_description": xacro.process_file(sim_urdf).toxml()}

    srdf = os.path.join(moveit_pkg, "config", "hitbot.srdf")
    with open(srdf, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics = {"robot_description_kinematics":
                  _load_yaml("hitbot_moveit_config", "config/kinematics.yaml")}
    joint_limits = _load_yaml("hitbot_moveit_config", "config/joint_limits.yaml")
    ompl_yaml = _load_yaml("hitbot_moveit_config", "config/ompl_planning.yaml")
    ompl_config = {"planning_pipelines": ["ompl"],
                   "default_planning_pipeline": "ompl", "ompl": ompl_yaml}
    moveit_controllers = _load_yaml("hitbot_moveit_config",
                                    "config/moveit_controllers.yaml")
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # --- nodes ------------------------------------------------------------ #
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[robot_description, use_sim_time])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"), "launch",
            "gazebo.launch.py")),
        launch_arguments={"gui": gui}.items())

    spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-entity", "hitbot", "-topic", "robot_description"],
                 output="screen")

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"], output="screen")
    load_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "arm_controller"], output="screen")

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics,
                    joint_limits, ompl_config, moveit_controllers,
                    trajectory_execution, planning_scene_monitor,
                    {"planning_plugin": "ompl_interface/OMPLPlanner"},
                    use_sim_time])

    rviz_cfg = os.path.join(moveit_pkg, "config", "moveit.rviz")
    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        condition=IfCondition(gui),
        arguments=(["-d", rviz_cfg] if os.path.exists(rviz_cfg) else []),
        parameters=[robot_description, robot_description_semantic, kinematics,
                    use_sim_time])

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo client + RViz GUI"),
        # chain controller loading after the robot is spawned
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[load_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=load_jsb,
                                           on_exit=[load_arm])),
        gazebo, rsp, spawn, move_group, rviz,
    ])
