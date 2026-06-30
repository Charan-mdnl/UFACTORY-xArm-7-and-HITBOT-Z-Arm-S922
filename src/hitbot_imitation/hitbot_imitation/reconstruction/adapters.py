"""Adapter interfaces for the perception foundation models (Stage A).

Each external model the paper uses is wrapped behind a small abstract role so
the orchestration in :mod:`pipeline` depends on *interfaces*, not specific
libraries.  Concrete adapters lazily import their backend, so importing this
module (and the whole retargeting/execution half of the package) never requires
the heavy dependencies to be installed.

Paper → role mapping
--------------------
    HaWoR          -> HandTracker        (per-frame wrist pose + hand mesh)
    SAM 3          -> Segmenter          (per-frame hand & object masks)
    MoGe           -> DepthEstimator     (metric depth, intrinsics, pointmap)
    SAM 3D         -> ObjectMesher       (single-image canonical object mesh)
    SAM 3D (flow)  -> FlowVelocityModel  (velocity field for guided tracking)
    BootsTAPIR     -> PointTracker        (2D point tracks for adaptive guidance)
    GeoCalib       -> GravityEstimator    (camera gravity direction)

Install these on the GPU machine (see ``requirements.txt`` / README) and pass
instances into :class:`~hitbot_imitation.reconstruction.pipeline.ReconstructionPipeline`.
"""
from __future__ import annotations

import abc

import numpy as np


class _MissingBackend:
    """Helper to raise a uniform, actionable error when a backend is absent."""

    @staticmethod
    def fail(role: str, package: str, exc: Exception) -> None:
        raise ImportError(
            f"{role} backend '{package}' is not installed in this environment.\n"
            f"Install it on the GPU machine (see hitbot_imitation/requirements.txt "
            f"and README 'Stage A' section), then retry. Original error: {exc}")


class HandTracker(abc.ABC):
    """video frames -> per-frame wrist pose + (optional) visible hand vertices."""

    @abc.abstractmethod
    def track(self, frames: np.ndarray) -> dict:
        """Return ``{"wrist_poses": (N,4,4), "vertices": (N,V,3)|None}``."""


class Segmenter(abc.ABC):
    """video frames -> per-frame binary masks for the hand and the object."""

    @abc.abstractmethod
    def segment(self, frames: np.ndarray) -> dict:
        """Return ``{"hand": (N,H,W) bool, "object": (N,H,W) bool}``."""


class DepthEstimator(abc.ABC):
    """single frame -> metric depth, camera intrinsics and a 3D pointmap."""

    @abc.abstractmethod
    def estimate(self, frame: np.ndarray) -> dict:
        """Return ``{"depth": (H,W), "K": (3,3), "pointmap": (H,W,3)}``."""


class ObjectMesher(abc.ABC):
    """anchor frame + object mask -> canonical object mesh (vertices, faces)."""

    @abc.abstractmethod
    def mesh(self, frame: np.ndarray, mask: np.ndarray) -> dict:
        """Return ``{"vertices": (V,3), "faces": (F,3)}`` in object frame."""


class FlowVelocityModel(abc.ABC):
    """SAM 3D's flow-matching backbone, restricted to the pose block.

    The guided tracker (:mod:`guided_tracking`) only needs to query the
    velocity field ``v_theta(x_t, t, c)`` for shape and pose blocks; this
    abstracts exactly that so the paper's Eq. 1 blending can be implemented
    without vendoring SAM 3D internals.
    """

    @abc.abstractmethod
    def velocity(self, x_shape: np.ndarray, x_pose: np.ndarray, t: float,
                 cond: dict) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(v_shape, v_pose)`` at flow time ``t`` given condition."""

    @abc.abstractmethod
    def sample_noise(self) -> tuple[np.ndarray, np.ndarray]:
        """Return fresh ``(eps_shape, eps_pose)`` initial noise."""


class PointTracker(abc.ABC):
    """consecutive frames + object mask -> 2D point tracks for guidance."""

    @abc.abstractmethod
    def track_points(self, frame_a: np.ndarray, frame_b: np.ndarray,
                     mask_a: np.ndarray) -> dict:
        """Return ``{"pa": (P,2), "pb": (P,2), "valid": (P,) bool}``."""


class GravityEstimator(abc.ABC):
    """single frame -> estimated up/gravity direction in camera coordinates."""

    @abc.abstractmethod
    def gravity(self, frame: np.ndarray) -> np.ndarray:
        """Return a (3,) unit up-vector in the camera frame."""


# --------------------------------------------------------------------------- #
# Concrete lazy-loading adapters.  These adapt each library's real output to
# the interface above.  The exact upstream call sites are documented inline;
# wire them to the installed package versions on the GPU machine.
# --------------------------------------------------------------------------- #
class HaWoRHandTracker(HandTracker):
    """Adapter for HaWoR world-space hand motion reconstruction."""

    def __init__(self, device: str = "cuda", **kw):
        try:
            import hawor  # type: ignore  # noqa: F401
        except Exception as exc:                       # pragma: no cover
            _MissingBackend.fail("HandTracker", "hawor", exc)
        self.device = device
        self._kw = kw

    def track(self, frames: np.ndarray) -> dict:        # pragma: no cover
        # Expected: run HaWoR to get per-frame MANO/wrist global pose. Map the
        # wrist global orientation+translation into a (N,4,4) and collect the
        # visible hand-mesh vertices used later for hand-object alignment.
        raise NotImplementedError(
            "Wire HaWoRHandTracker.track to your installed HaWoR API: produce "
            "wrist_poses (N,4,4) and per-frame hand vertices (N,V,3).")


class SAM3Segmenter(Segmenter):
    """Adapter for SAM 3 (Segment Anything with Concepts)."""

    def __init__(self, checkpoint: str | None = None, device: str = "cuda", **kw):
        try:
            import sam3  # type: ignore  # noqa: F401
        except Exception as exc:                       # pragma: no cover
            _MissingBackend.fail("Segmenter", "sam3", exc)
        self.device = device

    def segment(self, frames: np.ndarray) -> dict:      # pragma: no cover
        raise NotImplementedError(
            "Wire SAM3Segmenter.segment to SAM 3: return per-frame 'hand' and "
            "'object' boolean masks of shape (N,H,W).")


class MoGeDepthEstimator(DepthEstimator):
    """Adapter for MoGe-2 monocular metric geometry."""

    def __init__(self, device: str = "cuda", **kw):
        try:
            import moge  # type: ignore  # noqa: F401
        except Exception as exc:                       # pragma: no cover
            _MissingBackend.fail("DepthEstimator", "moge", exc)
        self.device = device

    def estimate(self, frame: np.ndarray) -> dict:      # pragma: no cover
        raise NotImplementedError(
            "Wire MoGeDepthEstimator.estimate to MoGe: return depth (H,W), "
            "intrinsics K (3,3) and pointmap (H,W,3).")


class SAM3DObjectMesher(ObjectMesher):
    """Adapter for SAM 3D single-image object meshing."""

    def __init__(self, device: str = "cuda", **kw):
        try:
            import sam3d  # type: ignore  # noqa: F401
        except Exception as exc:                       # pragma: no cover
            _MissingBackend.fail("ObjectMesher", "sam3d", exc)
        self.device = device

    def mesh(self, frame: np.ndarray, mask: np.ndarray) -> dict:  # pragma: no cover
        raise NotImplementedError(
            "Wire SAM3DObjectMesher.mesh to SAM 3D: return canonical object "
            "'vertices' (V,3) and 'faces' (F,3).")
