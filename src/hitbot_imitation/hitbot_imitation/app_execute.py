"""Stage C: execute an EETrajectory on the HITBOT via MoveIt + ros2_control.

Pipeline:  ee_traj.npz  --(MoveIt /compute_ik, seeded)-->  JointTrajectory
           --(FollowJointTrajectory action)-->  arm_controller  -->  Gazebo

Replay rather than re-planning: Stage B already produced a dense, smooth,
reachable Cartesian path, so per-waypoint IK seeded with the previous solution
gives a continuous joint trajectory streamed by the existing ``arm_controller``.
The reconstructed EE path is also published as an RViz Marker for the paper's
"digital twin" visual check (Fig. 11).

    ros2 run hitbot_imitation execute_moveit --ros-args \
        -p trajectory:=/path/to/ee_traj.npz -p dry_run:=false
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

from .utils import io, se3

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__("hitbot_imitation_executor")
        self.declare_parameter("trajectory", "")
        self.declare_parameter("group", "arm")
        self.declare_parameter("ik_link", "S922_7")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("ik_timeout", 0.05)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("time_scale", 1.0)   # >1 slows playback down

        self.group = self.get_parameter("group").value
        self.ik_link = self.get_parameter("ik_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.time_scale = float(self.get_parameter("time_scale").value)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory")
        self.marker_pub = self.create_publisher(Marker, "imitation_ee_path", 1)
        self._current_state = None
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        self._current_state = msg

    # -- IK ---------------------------------------------------------------- #
    def _seed_state(self, seed_positions) -> RobotState:
        rs = RobotState()
        js = JointState()
        js.name = list(JOINT_NAMES)
        js.position = list(seed_positions)
        rs.joint_state = js
        return rs

    def _pose_msg(self, T: np.ndarray) -> PoseStamped:
        q = se3.quat(T)               # [x,y,z,w]
        t = se3.translation(T)
        ps = PoseStamped()
        ps.header = Header(frame_id=self.base_frame)
        ps.pose = Pose(
            position=Point(x=float(t[0]), y=float(t[1]), z=float(t[2])),
            orientation=Quaternion(x=float(q[0]), y=float(q[1]),
                                   z=float(q[2]), w=float(q[3])))
        return ps

    def solve_ik(self, T: np.ndarray, seed):
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.group
        ik.ik_link_name = self.ik_link
        ik.robot_state = self._seed_state(seed)
        ik.pose_stamped = self._pose_msg(T)
        ik.avoid_collisions = False
        ik.timeout = Duration(sec=0, nanosec=int(self.ik_timeout * 1e9))
        req.ik_request = ik
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        res = future.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        name_to_pos = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
        out = [name_to_pos[j] for j in JOINT_NAMES if j in name_to_pos]
        return out if len(out) == 6 else None

    # -- build + send ------------------------------------------------------ #
    def build_joint_trajectory(self, ee) -> JointTrajectory:
        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)
        seed = [0.0] * 6
        if self._current_state is not None:
            cur = dict(zip(self._current_state.name,
                           self._current_state.position))
            seed = [cur.get(j, 0.0) for j in JOINT_NAMES]

        n_fail = 0
        for i in range(ee.n_frames):
            sol = self.solve_ik(ee.ee_poses[i], seed)
            if sol is None:
                n_fail += 1
                continue                      # skip unreachable waypoint
            seed = sol                        # continuity
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in sol]
            tt = float(ee.timestamps[i]) * self.time_scale
            pt.time_from_start = Duration(sec=int(tt),
                                          nanosec=int((tt % 1.0) * 1e9))
            jt.points.append(pt)
        self.get_logger().info(
            f"IK solved {len(jt.points)}/{ee.n_frames} waypoints "
            f"({n_fail} unreachable skipped)")
        return jt

    def publish_marker(self, ee) -> None:
        m = Marker()
        m.header = Header(frame_id=self.base_frame)
        m.ns = "imitation"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.005
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.6, 1.0, 1.0
        for i in range(ee.n_frames):
            t = se3.translation(ee.ee_poses[i])
            m.points.append(Point(x=float(t[0]), y=float(t[1]), z=float(t[2])))
        self.marker_pub.publish(m)

    def run(self) -> int:
        path = self.get_parameter("trajectory").value
        if not path:
            self.get_logger().error("set -p trajectory:=/path/to/ee_traj.npz")
            return 1
        ee = io.load_ee(path)
        self.get_logger().info(
            f"loaded EE trajectory: {ee.n_frames} pts, frame={ee.base_frame}, "
            f"link={ee.ee_link}, grasp closed on {int(ee.gripper.sum())} pts")
        self.publish_marker(ee)

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "/compute_ik unavailable — is move_group running?")
            return 1
        jt = self.build_joint_trajectory(ee)
        if not jt.points:
            self.get_logger().error("no reachable waypoints; aborting")
            return 1
        if self.dry_run:
            self.get_logger().info("dry_run: not sending to controller")
            return 0
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "arm_controller action unavailable — is the controller up?")
            return 1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        self.get_logger().info("sending trajectory to arm_controller ...")
        send = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh.accepted:
            self.get_logger().error("goal rejected by controller")
            return 1
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("execution complete")
        return 0


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = TrajectoryExecutor()
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
