"""Generate a synthetic hand-object trajectory (a stand-in for Stage A).

This lets the retargeting + MoveIt/Gazebo execution path be developed and
tested end-to-end before the perception foundation models are installed.  It
fabricates a simple pick-and-place: the object rests, a hand approaches and
grasps it, carries it along an arc to a new location, sets it down, and
retreats.  The output is a real :class:`HandObjectTrajectory`, identical in
shape to what the reconstruction pipeline produces.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..types import HandObjectTrajectory
from ..utils import se3


def make_pick_and_place(n: int = 120, fps: float = 30.0,
                        seed: int = 0) -> HandObjectTrajectory:
    """Build a pick-and-place hand-object trajectory with ``n`` frames."""
    rng = np.random.default_rng(seed)
    t = np.arange(n) / fps

    # Phase boundaries (approach / grasp+carry / release+retreat).
    g0 = n // 4          # grasp onset
    g1 = 3 * n // 4      # release

    pick = np.array([0.0, 0.0, 0.0])
    place = np.array([0.30, -0.15, 0.0])

    object_pos = np.tile(pick, (n, 1)).astype(float)
    # During the carry the object follows a raised arc from pick to place.
    carry = np.linspace(0.0, 1.0, g1 - g0)
    arc = np.sin(carry * np.pi) * 0.12          # lift in +Z
    object_pos[g0:g1] = (pick[None] + carry[:, None] * (place - pick)[None])
    object_pos[g0:g1, 2] += arc
    object_pos[g1:] = place

    # Object yaws as it is carried (e.g. reorienting to place).
    object_yaw = np.zeros(n)
    object_yaw[g0:g1] = carry * (np.pi / 3)
    object_yaw[g1:] = np.pi / 3

    # The hand approaches from above/behind, coincides with the object during
    # the grasp, then retreats upward.
    hand_pos = object_pos.copy()
    approach = np.linspace(0.0, 1.0, g0)
    hand_pos[:g0] = object_pos[:g0] + (1 - approach)[:, None] * np.array([-0.15, 0.0, 0.20])
    retreat = np.linspace(0.0, 1.0, n - g1)
    hand_pos[g1:] = object_pos[g1:] + retreat[:, None] * np.array([0.0, 0.0, 0.20])
    # Small grip offset so wrist sits just behind the object centroid.
    grip_off = np.array([-0.03, 0.0, 0.0])
    hand_pos[g0:g1] += grip_off

    # Hand orientation: gentle downward-facing approach that follows object yaw.
    object_poses = np.zeros((n, 4, 4))
    hand_poses = np.zeros((n, 4, 4))
    down = R.from_euler("y", np.pi / 2)          # tool pointing -X→down-ish
    for i in range(n):
        object_poses[i] = se3.make_pose(
            object_pos[i], R.from_euler("z", object_yaw[i]))
        hand_poses[i] = se3.make_pose(
            hand_pos[i], R.from_euler("z", object_yaw[i]) * down)

    # Add a little reconstruction-like noise.
    object_poses[:, :3, 3] += rng.normal(0, 0.002, (n, 3))
    hand_poses[:, :3, 3] += rng.normal(0, 0.002, (n, 3))

    contact = np.zeros(n, dtype=bool)
    contact[g0:g1] = True

    return HandObjectTrajectory(
        timestamps=t,
        object_poses=object_poses,
        hand_poses=hand_poses,
        contact=contact,
        object_mesh_path=None,
        gravity_camera=np.array([0.0, 0.0, 1.0]),
        meta={"source": "synthetic.make_pick_and_place", "fps": fps},
    )
