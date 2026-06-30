"""Map a human-space trajectory into the robot's workspace.

The reconstructed trajectory is metric and gravity-aligned, but it still lives
wherever the human performed the task relative to the camera.  The paper does a
manual ``(x, y, z, yaw)`` alignment of the initial pose to the robot workspace
before computing arm IK (Sec. 4.4).  This module makes that rigid placement
explicit and adds an optional uniform scale so a large human motion fits the
arm's reach.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R

from ..utils import se3


@dataclass
class WorkspaceConfig:
    """Where, in the robot base frame, to anchor the start of the motion.

    Attributes
    ----------
    anchor_xyz : (3,) world/base position to place the first EE point at.
    anchor_yaw : rotation (rad) about base +Z applied to the whole motion.
    scale : uniform scale on the *relative* motion about the anchor
        (1.0 keeps metric size; <1 shrinks a big human motion into reach).
    base_frame : name of the robot base frame.
    """

    anchor_xyz: tuple[float, float, float] = (0.35, 0.0, 0.30)
    anchor_yaw: float = 0.0
    scale: float = 1.0
    base_frame: str = "world"


def align_to_workspace(ee_poses: np.ndarray,
                       cfg: WorkspaceConfig) -> np.ndarray:
    """Rigidly + uniformly place an EE pose trajectory into the robot frame.

    The first pose is moved to ``cfg.anchor_xyz``; the whole trajectory is
    yaw-rotated by ``cfg.anchor_yaw`` about that anchor and scaled by
    ``cfg.scale`` relative to it.  Orientation of each pose is yaw-rotated to
    stay consistent with the translated path.
    """
    ee_poses = np.asarray(ee_poses, dtype=float).copy()
    if len(ee_poses) == 0:
        return ee_poses

    p0 = se3.translation(ee_poses[0]).copy()
    yaw = R.from_euler("z", cfg.anchor_yaw).as_matrix()
    anchor = np.asarray(cfg.anchor_xyz, dtype=float)

    trans = se3.translation(ee_poses)                     # (N,3)
    rel = (trans - p0) * cfg.scale                        # about first point
    new_trans = (yaw @ rel.T).T + anchor                  # rotate + place

    rots = se3.rotation_matrix(ee_poses)                  # (N,3,3)
    new_rots = np.einsum("ij,njk->nik", yaw, rots)        # yaw-consistent

    ee_poses[:, :3, :3] = new_rots
    ee_poses[:, :3, 3] = new_trans
    return ee_poses


def reach_report(ee_poses: np.ndarray, base_xyz=(0.0, 0.0, 0.0),
                 max_reach: float = 0.92) -> dict:
    """Quick reachability sanity check against a spherical reach bound.

    The S922 has a finite reach; this flags points outside a coarse sphere so
    the operator can lower ``scale`` / move the anchor before sending to IK.
    ``max_reach`` defaults to a conservative S922 envelope (metres).
    """
    base = np.asarray(base_xyz, dtype=float)
    d = np.linalg.norm(se3.translation(ee_poses) - base, axis=1)
    n_out = int(np.sum(d > max_reach))
    return {
        "min_dist": float(d.min()),
        "max_dist": float(d.max()),
        "n_out_of_reach": n_out,
        "fraction_out": float(n_out) / len(d) if len(d) else 0.0,
        "max_reach": max_reach,
    }
