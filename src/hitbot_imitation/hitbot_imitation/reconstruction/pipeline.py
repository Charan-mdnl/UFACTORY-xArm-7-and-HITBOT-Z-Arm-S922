"""Stage A orchestration: monocular RGB video -> HandObjectTrajectory.

Implements the reconstruction flow of paper Section 3.1 / Appendix A on top of
the pluggable adapters.  The orchestration logic (ordering, guided tracking,
alignment, contact) is real; the heavy per-model inference is delegated to the
adapters you wire up on the GPU machine.

Typical use (on the GPU laptop, once dependencies are installed)::

    from hitbot_imitation.reconstruction import pipeline as P
    from hitbot_imitation.reconstruction import adapters as A

    pipe = P.ReconstructionPipeline(
        hand_tracker=A.HaWoRHandTracker(),
        segmenter=A.SAM3Segmenter(),
        depth=A.MoGeDepthEstimator(),
        mesher=A.SAM3DObjectMesher(),
        flow=my_sam3d_flow_model,            # FlowVelocityModel
        point_tracker=my_bootstapir,         # PointTracker (optional)
        gravity=my_geocalib,                 # GravityEstimator (optional)
    )
    traj = pipe.run("cooking_clip.mp4")
    from hitbot_imitation.utils.io import save_handobj
    save_handobj(traj, "handobj_traj.npz")
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..types import HandObjectTrajectory
from ..utils import se3
from . import alignment
from .guided_tracking import (GuidanceConfig, adaptive_alpha_p,
                              integrate_guided, rotation_from_point_tracks,
                              se3_cluster_select)


def read_video(path: str, max_frames: int | None = None,
               stride: int = 1) -> tuple[np.ndarray, float]:
    """Read frames from a video into ``(N,H,W,3)`` uint8 and return (frames, fps)."""
    try:
        import cv2
    except Exception as exc:                            # pragma: no cover
        raise ImportError("OpenCV (cv2) is required to read video.") from exc
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames, i = [], 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if i % stride == 0:
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if max_frames and len(frames) >= max_frames:
                break
        i += 1
    cap.release()
    return np.asarray(frames), fps / stride


@dataclass
class ReconstructionConfig:
    anchor_frame: int = 0
    contact_threshold: float = 0.05    # metres, hand-object distance for "in-hand"
    guidance: GuidanceConfig = None    # type: ignore

    def __post_init__(self):
        if self.guidance is None:
            self.guidance = GuidanceConfig()


class ReconstructionPipeline:
    """Run the full Stage A reconstruction given wired model adapters."""

    def __init__(self, hand_tracker, segmenter, depth, mesher, flow,
                 point_tracker=None, gravity=None,
                 cfg: ReconstructionConfig | None = None):
        self.hand_tracker = hand_tracker
        self.segmenter = segmenter
        self.depth = depth
        self.mesher = mesher
        self.flow = flow
        self.point_tracker = point_tracker
        self.gravity = gravity
        self.cfg = cfg or ReconstructionConfig()

    # -- object tracking via guided diffusion (Sec 3.1) -------------------- #
    def _track_object(self, frames, obj_masks, depths) -> tuple[np.ndarray, np.ndarray]:
        """Return per-frame object (orientations (N,3,3), cam translations (N,3))."""
        n = len(frames)
        g = self.cfg.guidance

        # Fix canonical shape at the anchor frame.
        anchor = self.cfg.anchor_frame
        shape_anchor = self.flow.encode_shape(frames[anchor], obj_masks[anchor]) \
            if hasattr(self.flow, "encode_shape") else None

        orientations = np.tile(np.eye(3), (n, 1, 1))
        translations = np.zeros((n, 3))
        pose_prev = self.flow.initial_pose(frames[anchor], obj_masks[anchor]) \
            if hasattr(self.flow, "initial_pose") else np.zeros(0)

        for k in range(n):
            # adaptive pose guidance from 2D point-track rotation
            alpha_p = 0.5
            if self.point_tracker is not None and k > 0:
                tr = self.point_tracker.track_points(
                    frames[k - 1], frames[k], obj_masks[k - 1])
                dtheta = rotation_from_point_tracks(tr["pa"], tr["pb"],
                                                    tr.get("valid"))
                alpha_p = adaptive_alpha_p(dtheta)

            cond = {"image": frames[k], "mask": obj_masks[k], "depth": depths[k]}
            candidates = []
            for _ in range(g.n_candidates):
                x_pose = integrate_guided(self.flow, cond, shape_anchor,
                                          pose_prev, alpha_p, g)
                candidates.append(self.flow.decode_pose(x_pose))   # -> (4,4)
            pose = se3_cluster_select(candidates, g)
            orientations[k] = se3.rotation_matrix(pose)
            translations[k] = se3.translation(pose)
            pose_prev = self.flow.encode_pose(pose) \
                if hasattr(self.flow, "encode_pose") else pose_prev
        return orientations, translations

    def run(self, video_path: str, max_frames: int | None = None,
            stride: int = 1) -> HandObjectTrajectory:
        frames, fps = read_video(video_path, max_frames, stride)
        n = len(frames)
        t = np.arange(n) / fps

        masks = self.segmenter.segment(frames)
        depths = [self.depth.estimate(f) for f in frames]
        depth_maps = [d["depth"] for d in depths]

        hand = self.hand_tracker.track(frames)
        hand_poses = hand["wrist_poses"]

        orientations, cam_trans = self._track_object(frames, masks["object"],
                                                     depth_maps)

        # --- hand-object alignment (Appendix A) --------------------------- #
        c_hand_H = se3.translation(hand_poses)                  # near-metric hand
        # Object/hand centroids in pointmap space from depth + masks.
        c_obj_M = np.stack([_masked_centroid(depths[i]["pointmap"], masks["object"][i])
                            for i in range(n)])
        c_hand_M = np.stack([_masked_centroid(depths[i]["pointmap"], masks["hand"][i])
                             for i in range(n)])
        k = alignment.per_frame_scale(c_hand_H[:, 2], c_hand_M[:, 2])
        obj_target = alignment.object_target_position(c_hand_H, c_obj_M, c_hand_M, k)

        object_poses = alignment.assemble_object_poses(orientations, obj_target)

        # --- gravity alignment (GeoCalib) --------------------------------- #
        up = (self.gravity.gravity(frames[0]) if self.gravity is not None
              else np.array([0.0, -1.0, 0.0]))
        object_poses, hand_poses = alignment.gravity_align(object_poses,
                                                           hand_poses, up)

        contact = alignment.contact_from_distance(object_poses, hand_poses,
                                                  self.cfg.contact_threshold)

        return HandObjectTrajectory(
            timestamps=t, object_poses=object_poses, hand_poses=hand_poses,
            contact=contact, gravity_camera=up,
            meta={"source": video_path, "fps": float(fps), "n_frames": n})


def _masked_centroid(pointmap: np.ndarray, mask: np.ndarray) -> np.ndarray:
    """Mean 3D point over a boolean mask in a ``(H,W,3)`` pointmap."""
    pts = pointmap[mask.astype(bool)]
    if len(pts) == 0:
        return np.zeros(3)
    return pts.mean(0)
