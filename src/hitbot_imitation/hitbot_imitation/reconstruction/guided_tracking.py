"""Guided-diffusion object tracking (paper Section 3.1 + Appendix A).

This is the paper's actual algorithmic contribution and is implemented here in
NumPy, parameterised by a :class:`~.adapters.FlowVelocityModel` (SAM 3D's flow
backbone).  It turns a per-frame image-to-3D generator into a temporally
coherent 6-DoF object tracker by:

1. fixing the canonical shape at an anchor frame,
2. integrating the flow ODE while *blending* toward the canonical shape and the
   previous frame's pose at every Euler step (Eq. 1),
3. setting the pose-guidance strength adaptively from 2D point-track rotation
   (Appendix A),
4. drawing N candidate poses and selecting the mode via SE(3) clustering
   (Eq. 3), avoiding the prohibitive exact log-density ranking.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from ..utils import se3


@dataclass
class GuidanceConfig:
    n_steps: int = 25          # Euler steps T
    n_candidates: int = 25     # candidates N per frame
    alpha_s: float = 0.95      # fixed shape guidance (rigid objects: [0.9,1])
    w_t: float = 1.0           # SE(3) clustering translation weight
    w_r: float = 0.1           # SE(3) clustering rotation weight
    cluster_eps: float = 0.05  # cluster radius under the weighted SE(3) metric
    min_cluster_frac: float = 0.15  # discard clusters smaller than this


def adaptive_alpha_p(delta_theta: float) -> float:
    """Pose guidance strength from in-plane rotation magnitude (Appendix A).

    ``alpha_p(k) = max(0.1, 0.7 - 0.09 * |delta_theta|)`` with ``delta_theta``
    in radians.  Large rotation -> trust the model more (lower blending);
    near-static -> blend strongly toward the previous pose to suppress jitter.
    """
    return float(max(0.1, 0.7 - 0.09 * abs(delta_theta)))


def rotation_from_point_tracks(pa: np.ndarray, pb: np.ndarray,
                               valid: np.ndarray | None = None) -> float:
    """Closed-form in-plane rotation between matched 2D point sets via SVD.

    Fits a 2D rigid transform (Procrustes) ``pb ~ Rot @ pa + t`` and returns the
    rotation angle.  Used to drive :func:`adaptive_alpha_p`.
    """
    pa = np.asarray(pa, dtype=float)
    pb = np.asarray(pb, dtype=float)
    if valid is not None:
        pa, pb = pa[valid], pb[valid]
    if len(pa) < 2:
        return 0.0
    ca, cb = pa.mean(0), pb.mean(0)
    H = (pa - ca).T @ (pb - cb)
    U, _, Vt = np.linalg.svd(H)
    Rm = Vt.T @ U.T
    if np.linalg.det(Rm) < 0:                 # reflection fix
        Vt[-1] *= -1
        Rm = Vt.T @ U.T
    return float(np.arctan2(Rm[1, 0], Rm[0, 0]))


def integrate_guided(flow, cond: dict, shape_anchor: np.ndarray,
                     pose_prev: np.ndarray, alpha_p: float,
                     cfg: GuidanceConfig) -> np.ndarray:
    """Integrate one guided sample of the pose block (Eq. 1).

    ``flow`` is a :class:`~.adapters.FlowVelocityModel`.  Returns the final
    pose-block latent ``x_pose`` (decode to SE(3) with the model's decoder).
    Shape is held at ``shape_anchor`` via strong fixed guidance; pose is nudged
    toward ``pose_prev`` with strength ``alpha_p``.
    """
    eps_s, eps_p = flow.sample_noise()
    x_s, x_p = eps_s.copy(), eps_p.copy()
    dt = 1.0 / cfg.n_steps
    for i in range(cfg.n_steps):
        t = i * dt
        v_s, v_p = flow.velocity(x_s, x_p, t, cond)
        # target interpolants along the model's own probability path
        z_s = (1 - t) * eps_s + t * shape_anchor
        z_p = (1 - t) * eps_p + t * pose_prev
        x_s = (1 - cfg.alpha_s) * (x_s + dt * v_s) + cfg.alpha_s * z_s
        x_p = (1 - alpha_p) * (x_p + dt * v_p) + alpha_p * z_p
    return x_p


def se3_cluster_select(candidates: list[np.ndarray], cfg: GuidanceConfig,
                       iou_fn: Callable[[np.ndarray], float] | None = None
                       ) -> np.ndarray:
    """Select the mode-best pose from candidates via SE(3) clustering (Eq. 3).

    Greedy clustering under the weighted SE(3) metric; clusters below
    ``min_cluster_frac * N`` are discarded as outliers, and the surviving
    clusters are ranked by silhouette/mask IoU (``iou_fn``) when available,
    otherwise by cluster size.  Returns the medoid pose of the winning cluster.
    """
    n = len(candidates)
    if n == 0:
        raise ValueError("no candidates to select from")
    if n == 1:
        return candidates[0]

    # pairwise weighted SE(3) distances
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            d = se3.se3_distance(candidates[i], candidates[j],
                                 w_t=cfg.w_t, w_r=cfg.w_r)
            D[i, j] = D[j, i] = d

    # greedy clustering by radius
    unassigned = set(range(n))
    clusters: list[list[int]] = []
    while unassigned:
        seed = min(unassigned)
        members = [k for k in unassigned if D[seed, k] <= cfg.cluster_eps]
        clusters.append(members)
        unassigned -= set(members)

    min_size = max(1, int(cfg.min_cluster_frac * n))
    clusters = [c for c in clusters if len(c) >= min_size] or clusters

    def score(cluster: list[int]) -> float:
        if iou_fn is not None:
            return max(iou_fn(candidates[k]) for k in cluster)
        return float(len(cluster))

    best = max(clusters, key=score)
    # medoid = member minimising total intra-cluster distance
    medoid = min(best, key=lambda k: sum(D[k, m] for m in best))
    return candidates[medoid]
