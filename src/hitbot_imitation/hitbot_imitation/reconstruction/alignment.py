"""Hand-object alignment and gravity alignment (paper Section 3.1, Appendix A).

Hand (HaWoR) and object (SAM 3D / MoGe pointmap) are reconstructed at possibly
different scales and origins.  We treat the hand scale as near-metric ground
truth and slide/scale the object onto it, then align the whole trajectory with
gravity.  All pure NumPy.
"""
from __future__ import annotations

import numpy as np

from ..utils import se3


def per_frame_scale(z_hand_H: np.ndarray, z_hand_M: np.ndarray) -> np.ndarray:
    """Scale ratio ``k = z^H_hand / z^M_hand`` between hand-mesh and pointmap.

    ``z_*`` are the depth (camera-z) components of the hand centroid in the two
    spaces.  Clamps near-zero denominators for stability.
    """
    z_hand_M = np.asarray(z_hand_M, dtype=float)
    denom = np.where(np.abs(z_hand_M) < 1e-6, 1e-6, z_hand_M)
    return np.asarray(z_hand_H, dtype=float) / denom


def object_target_position(c_hand_H: np.ndarray, c_obj_M: np.ndarray,
                           c_hand_M: np.ndarray, k: np.ndarray) -> np.ndarray:
    """``obj_target = c^H_hand + k (c^M_obj - c^M_hand)`` (per frame).

    Places the object relative to the (near-metric) hand by rescaling the
    hand→object offset from pointmap space into hand-mesh units.
    """
    c_hand_H = np.asarray(c_hand_H, dtype=float)
    c_obj_M = np.asarray(c_obj_M, dtype=float)
    c_hand_M = np.asarray(c_hand_M, dtype=float)
    k = np.asarray(k, dtype=float).reshape(-1, 1)
    return c_hand_H + k * (c_obj_M - c_hand_M)


def solve_translation_scale(t_cam: np.ndarray, c_mesh: np.ndarray,
                            obj_target: np.ndarray) -> float:
    """1-D least-squares slide of the object along its viewing ray (Appendix A).

    ``objpos(s) = c_mesh + s * t_cam``; solve
    ``s* = t^T (obj_target - c_mesh) / (t^T t)`` so the object sits at the depth
    best matching ``obj_target`` while keeping its recovered orientation.
    """
    t_cam = np.asarray(t_cam, dtype=float)
    num = float(t_cam @ (np.asarray(obj_target) - np.asarray(c_mesh)))
    den = float(t_cam @ t_cam) + 1e-12
    return num / den


def assemble_object_poses(orientations: np.ndarray,
                          positions: np.ndarray) -> np.ndarray:
    """Combine per-frame rotation matrices and aligned positions into (N,4,4)."""
    orientations = np.asarray(orientations, dtype=float)
    positions = np.asarray(positions, dtype=float)
    n = len(positions)
    out = np.tile(np.eye(4), (n, 1, 1))
    out[:, :3, :3] = orientations
    out[:, :3, 3] = positions
    return out


def gravity_align(object_poses: np.ndarray, hand_poses: np.ndarray,
                  up_camera: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rotate both pose streams so the estimated up-vector maps to world +Z.

    Thin wrapper over :func:`hitbot_imitation.utils.se3.align_gravity` applied
    consistently to the object and the hand (GeoCalib step).
    """
    obj = se3.align_gravity(object_poses, up_camera)
    hand = se3.align_gravity(hand_poses, up_camera)
    return obj, hand


def contact_from_distance(object_poses: np.ndarray, hand_poses: np.ndarray,
                          threshold: float) -> np.ndarray:
    """Per-frame contact flag from hand-object centroid distance < threshold.

    Mirrors the paper's reference-stage definition (Sec. 3.2): frames where the
    hand and object are within ``threshold`` metres are treated as in-hand.
    """
    d = np.linalg.norm(se3.translation(object_poses) - se3.translation(hand_poses),
                       axis=1)
    return d < threshold
