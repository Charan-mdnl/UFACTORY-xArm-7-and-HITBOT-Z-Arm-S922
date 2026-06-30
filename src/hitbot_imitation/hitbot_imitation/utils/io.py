"""Load/save the intermediate trajectory artifacts as ``.npz`` files.

Keeping the boundary on disk lets you run Stage A on the GPU laptop, copy a
small ``.npz`` over, and iterate on Stage B/C anywhere.
"""
from __future__ import annotations

import json

import numpy as np

from ..types import EETrajectory, HandObjectTrajectory


def save_handobj(traj: HandObjectTrajectory, path: str) -> None:
    np.savez_compressed(
        path,
        timestamps=traj.timestamps,
        object_poses=traj.object_poses,
        hand_poses=traj.hand_poses,
        contact=traj.contact,
        object_mesh_path=traj.object_mesh_path or "",
        gravity_camera=(traj.gravity_camera
                        if traj.gravity_camera is not None
                        else np.full(3, np.nan)),
        meta=json.dumps(traj.meta),
    )


def load_handobj(path: str) -> HandObjectTrajectory:
    d = np.load(path, allow_pickle=False)
    grav = d["gravity_camera"]
    return HandObjectTrajectory(
        timestamps=d["timestamps"],
        object_poses=d["object_poses"],
        hand_poses=d["hand_poses"],
        contact=d["contact"],
        object_mesh_path=str(d["object_mesh_path"]) or None,
        gravity_camera=None if np.isnan(grav).all() else grav,
        meta=json.loads(str(d["meta"])),
    )


def save_ee(traj: EETrajectory, path: str) -> None:
    np.savez_compressed(
        path,
        timestamps=traj.timestamps,
        ee_poses=traj.ee_poses,
        gripper=traj.gripper,
        base_frame=traj.base_frame,
        ee_link=traj.ee_link,
        meta=json.dumps(traj.meta),
    )


def load_ee(path: str) -> EETrajectory:
    d = np.load(path, allow_pickle=False)
    return EETrajectory(
        timestamps=d["timestamps"],
        ee_poses=d["ee_poses"],
        gripper=d["gripper"],
        base_frame=str(d["base_frame"]),
        ee_link=str(d["ee_link"]),
        meta=json.loads(str(d["meta"])),
    )
