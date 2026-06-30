"""Shared data structures that flow between the pipeline stages.

The two stages of "Do as I Do" are decoupled at the ``HandObjectTrajectory``
boundary so the heavy perception stack (Stage A) and the robot-side retargeting
+ execution (Stage B/C) can be developed and tested independently.

    video ──Stage A (reconstruction)──▶ HandObjectTrajectory
          ──Stage B (retargeting)────▶ EETrajectory
          ──Stage C (execution)──────▶ MoveIt / Gazebo
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class HandObjectTrajectory:
    """A reconstructed 4D hand-object interaction (output of Stage A).

    All poses are 4x4 homogeneous transforms in a single, gravity-aligned
    metric world frame (camera-derived).  This is the embodiment-agnostic
    intermediate representation — the same file feeds a dexterous hand or,
    in our case, a 6-DOF arm.

    Attributes
    ----------
    timestamps : (N,) float seconds, monotonically increasing.
    object_poses : (N, 4, 4) object pose per frame.
    hand_poses : (N, 4, 4) wrist/palm pose per frame (HaWoR frame).
    contact : (N,) bool, True when the hand is grasping the object
        (reference hand-object distance under threshold; paper Sec. 3.2).
    object_mesh_path : optional path to the canonical object mesh (SAM3D).
    gravity_camera : (3,) estimated up direction in the original camera frame
        (GeoCalib); kept for provenance/debugging.
    meta : free-form provenance (source video, model versions, ...).
    """

    timestamps: np.ndarray
    object_poses: np.ndarray
    hand_poses: np.ndarray
    contact: np.ndarray
    object_mesh_path: str | None = None
    gravity_camera: np.ndarray | None = None
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.timestamps = np.asarray(self.timestamps, dtype=float)
        self.object_poses = np.asarray(self.object_poses, dtype=float)
        self.hand_poses = np.asarray(self.hand_poses, dtype=float)
        self.contact = np.asarray(self.contact, dtype=bool)
        n = len(self.timestamps)
        for name, arr, shape in (
            ("object_poses", self.object_poses, (n, 4, 4)),
            ("hand_poses", self.hand_poses, (n, 4, 4)),
            ("contact", self.contact, (n,)),
        ):
            if arr.shape != shape:
                raise ValueError(
                    f"{name} has shape {arr.shape}, expected {shape}")

    @property
    def n_frames(self) -> int:
        return len(self.timestamps)

    @property
    def duration(self) -> float:
        return float(self.timestamps[-1] - self.timestamps[0])


@dataclass
class EETrajectory:
    """An end-effector trajectory for the robot (output of Stage B).

    Attributes
    ----------
    timestamps : (N,) seconds.
    ee_poses : (N, 4, 4) target tool-flange poses in the robot base frame.
    gripper : (N,) float in [0, 1], 0 = open, 1 = closed (derived from
        ``HandObjectTrajectory.contact``; only actuated if a gripper exists).
    base_frame : name of the frame ``ee_poses`` are expressed in.
    ee_link : the robot link the poses command (tool flange).
    meta : provenance.
    """

    timestamps: np.ndarray
    ee_poses: np.ndarray
    gripper: np.ndarray
    base_frame: str = "world"
    ee_link: str = "S922_7"
    meta: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.timestamps = np.asarray(self.timestamps, dtype=float)
        self.ee_poses = np.asarray(self.ee_poses, dtype=float)
        self.gripper = np.asarray(self.gripper, dtype=float)

    @property
    def n_frames(self) -> int:
        return len(self.timestamps)
