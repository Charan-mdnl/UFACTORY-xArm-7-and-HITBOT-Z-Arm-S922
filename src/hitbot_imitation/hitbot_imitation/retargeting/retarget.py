"""Stage B: retarget a reconstructed hand-object trajectory onto the 6-DOF arm.

Embodiment note
---------------
The paper retargets human *finger* articulation onto a 22-DoF dexterous hand.
The HITBOT S922 has no fingers, so we retarget at the level the arm can
reproduce: the **wrist / grasped-object pose**.  The human wrist is the natural
analogue of the tool flange, and the fingers collapse into a 1-DoF gripper
open/close driven by the contact signal.

Within a grasp the wrist and object are (approximately) rigidly linked, so we
drive the tool from the *object* pose during contact — that reproduces the task
motion (pour/stir/place) exactly — and from the *wrist* pose during the
approach/retreat phases.  The two are stitched at the grasp onset by computing,
per segment, a constant grip transform ``T_ee_obj`` that makes them coincide,
so the resulting path is continuous.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..types import EETrajectory, HandObjectTrajectory
from ..utils import se3
from . import grasp as grasp_mod
from .workspace import WorkspaceConfig, align_to_workspace, reach_report


@dataclass
class RetargetConfig:
    """Parameters controlling the object→arm retargeting."""

    control_rate: float = 50.0          # Hz, matches the paper's 50 Hz commands
    hand_to_ee: np.ndarray = field(default_factory=lambda: np.eye(4))
    object_anchored_grasp: bool = True  # drive tool from object while grasping
    smooth_window: int = 7              # moving-average window on translations
    min_grasp_len: int = 2
    gripper_lead: int = 2
    ee_link: str = "S922_7"
    workspace: WorkspaceConfig = field(default_factory=WorkspaceConfig)


def _grip_transform(hand_g: np.ndarray, obj_g: np.ndarray,
                    hand_to_ee: np.ndarray) -> np.ndarray:
    """Constant ``T_ee_obj`` so object- and wrist-driven EE coincide at onset.

    Solve  ``T_obj_g @ inv(T_ee_obj) == T_hand_g @ T_hand_ee``  for T_ee_obj:
        T_ee_obj = inv(T_hand_ee) @ inv(T_hand_g) @ T_obj_g
    """
    return se3.compose(se3.inverse(hand_to_ee),
                       se3.inverse(hand_g), obj_g)


def retarget(traj: HandObjectTrajectory,
             cfg: RetargetConfig | None = None) -> EETrajectory:
    """Convert a :class:`HandObjectTrajectory` into an :class:`EETrajectory`."""
    cfg = cfg or RetargetConfig()
    hand_to_ee = np.asarray(cfg.hand_to_ee, dtype=float)

    # --- 1. base EE path = wrist tracking everywhere ---------------------
    ee = np.einsum("nij,jk->nik", traj.hand_poses, hand_to_ee)

    # --- 2. object-anchored override during each grasp segment -----------
    segments = grasp_mod.detect_segments(traj.contact, cfg.min_grasp_len)
    if cfg.object_anchored_grasp:
        for seg in segments:
            T_ee_obj = _grip_transform(traj.hand_poses[seg.start],
                                       traj.object_poses[seg.start],
                                       hand_to_ee)
            inv_grip = se3.inverse(T_ee_obj)
            for k in range(seg.start, seg.end + 1):
                ee[k] = traj.object_poses[k] @ inv_grip

    # --- 3. gripper command ---------------------------------------------
    gripper = grasp_mod.contact_to_gripper(traj.contact, segments,
                                           lead=cfg.gripper_lead)

    # --- 4. place into the robot workspace -------------------------------
    ee = align_to_workspace(ee, cfg.workspace)

    # --- 5. smooth, then resample to the control rate --------------------
    ee = se3.smooth_translations(ee, window=cfg.smooth_window)
    src_t = traj.timestamps - traj.timestamps[0]
    dt = 1.0 / cfg.control_rate
    dst_t = np.arange(0.0, src_t[-1] + 1e-9, dt)
    ee_rs = se3.interpolate_poses(ee, src_t, dst_t)
    grip_rs = np.interp(dst_t, src_t, gripper)
    grip_rs = (grip_rs > 0.5).astype(float)               # crisp open/close

    meta = dict(traj.meta)
    meta.update({
        "stage": "retarget",
        "n_segments": len(segments),
        "control_rate": cfg.control_rate,
        "reach": reach_report(ee_rs),
    })
    return EETrajectory(
        timestamps=dst_t,
        ee_poses=ee_rs,
        gripper=grip_rs,
        base_frame=cfg.workspace.base_frame,
        ee_link=cfg.ee_link,
        meta=meta,
    )
