"""SE(3) / rigid-transform utilities used across reconstruction and retargeting.

Everything is plain NumPy + SciPy so the retargeting/execution half of the
pipeline runs without any of the heavy perception dependencies installed.

Conventions
-----------
* A pose is a 4x4 homogeneous transform ``T = [[R, t], [0, 1]]``.
* Rotations are right-handed; quaternions are ``[x, y, z, w]`` (SciPy order).
* A "trajectory" of poses is an ``(N, 4, 4)`` array.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def make_pose(translation, rotation) -> np.ndarray:
    """Build a 4x4 pose from a 3-vector and a rotation.

    ``rotation`` may be a 3x3 matrix, a quaternion ``[x,y,z,w]`` or a
    :class:`scipy.spatial.transform.Rotation`.
    """
    T = np.eye(4)
    if isinstance(rotation, R):
        T[:3, :3] = rotation.as_matrix()
    else:
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape == (3, 3):
            T[:3, :3] = rotation
        elif rotation.shape == (4,):
            T[:3, :3] = R.from_quat(rotation).as_matrix()
        else:
            raise ValueError(f"unsupported rotation shape {rotation.shape}")
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T


def translation(T: np.ndarray) -> np.ndarray:
    return np.asarray(T)[..., :3, 3]


def rotation_matrix(T: np.ndarray) -> np.ndarray:
    return np.asarray(T)[..., :3, :3]


def quat(T: np.ndarray) -> np.ndarray:
    """Return the ``[x,y,z,w]`` quaternion of a single 4x4 pose."""
    return R.from_matrix(np.asarray(T)[:3, :3]).as_quat()


def inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a single 4x4 rigid transform."""
    Rm = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = Rm.T
    Ti[:3, 3] = -Rm.T @ t
    return Ti


def compose(*transforms: np.ndarray) -> np.ndarray:
    """Left-to-right composition ``A @ B @ C`` of 4x4 transforms."""
    out = np.eye(4)
    for T in transforms:
        out = out @ T
    return out


def relative_pose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pose of ``b`` expressed in the frame of ``a`` ( = inverse(a) @ b )."""
    return inverse(a) @ b


def geodesic_angle(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Geodesic angle (rad) between two rotation matrices."""
    rel = Ra.T @ Rb
    cos = (np.trace(rel) - 1.0) / 2.0
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))


def se3_distance(Ta: np.ndarray, Tb: np.ndarray, w_t: float = 1.0,
                 w_r: float = 0.1) -> float:
    """Weighted SE(3) distance used by the paper's clustering heuristic (Eq. 3)."""
    dt = np.linalg.norm(translation(Ta) - translation(Tb))
    dr = geodesic_angle(rotation_matrix(Ta), rotation_matrix(Tb))
    return w_t * dt + w_r * dr


def interpolate_poses(poses: np.ndarray, src_t: np.ndarray,
                      dst_t: np.ndarray) -> np.ndarray:
    """Resample an ``(N,4,4)`` pose trajectory onto new timestamps.

    Translations are linearly interpolated; rotations use SLERP.  Used for
    resampling reconstructed trajectories to the robot control rate.
    """
    poses = np.asarray(poses)
    src_t = np.asarray(src_t, dtype=float)
    dst_t = np.asarray(dst_t, dtype=float)
    dst_t = np.clip(dst_t, src_t[0], src_t[-1])

    trans = translation(poses)
    out_trans = np.vstack([np.interp(dst_t, src_t, trans[:, i]) for i in range(3)]).T

    key_rots = R.from_matrix(rotation_matrix(poses))
    slerp = Slerp(src_t, key_rots)
    out_rots = slerp(dst_t).as_matrix()

    out = np.tile(np.eye(4), (len(dst_t), 1, 1))
    out[:, :3, :3] = out_rots
    out[:, :3, 3] = out_trans
    return out


def smooth_translations(poses: np.ndarray, window: int = 5) -> np.ndarray:
    """Moving-average smoothing of the translation part of a pose trajectory.

    Rotations are left untouched (smoothing them well needs SO(3) filtering;
    the reconstructed rotations are already SLERP-resampled).  ``window`` is
    clamped to an odd value <= N.
    """
    poses = np.asarray(poses).copy()
    n = len(poses)
    if n < 3:
        return poses
    window = max(1, min(window, n if n % 2 else n - 1))
    if window % 2 == 0:
        window += 1
    half = window // 2
    trans = translation(poses)
    padded = np.pad(trans, ((half, half), (0, 0)), mode="edge")
    kernel = np.ones(window) / window
    sm = np.vstack([np.convolve(padded[:, i], kernel, mode="valid")
                    for i in range(3)]).T
    poses[:, :3, 3] = sm
    return poses


def align_gravity(poses: np.ndarray, up_camera: np.ndarray) -> np.ndarray:
    """Rotate a whole trajectory so ``up_camera`` maps to world +Z.

    Mirrors the paper's GeoCalib gravity-alignment step (Sec. 3.1): the
    reconstructed trajectory lives in camera coordinates, and we rotate the
    entire thing so the estimated gravity/up direction is vertical.
    """
    up = np.asarray(up_camera, dtype=float)
    up = up / (np.linalg.norm(up) + 1e-12)
    target = np.array([0.0, 0.0, 1.0])
    axis = np.cross(up, target)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return np.asarray(poses).copy()
    axis = axis / s
    angle = np.arccos(np.clip(np.dot(up, target), -1.0, 1.0))
    G = np.eye(4)
    G[:3, :3] = R.from_rotvec(axis * angle).as_matrix()
    return np.einsum("ij,njk->nik", G, np.asarray(poses))
