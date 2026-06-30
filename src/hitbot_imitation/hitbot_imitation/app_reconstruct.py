"""Stage A entry point: monocular RGB video -> HandObjectTrajectory (.npz).

Requires the perception foundation models (see requirements.txt / README
'Stage A').  Wires the adapters and runs the orchestration; edit
:func:`build_pipeline` to match the exact APIs/checkpoints you installed.
"""
from __future__ import annotations

import argparse

from .reconstruction import adapters as A
from .reconstruction import pipeline as P
from .utils import io


def build_pipeline(device: str) -> "P.ReconstructionPipeline":
    """Instantiate the model adapters. Wire these to your installed packages."""
    return P.ReconstructionPipeline(
        hand_tracker=A.HaWoRHandTracker(device=device),
        segmenter=A.SAM3Segmenter(device=device),
        depth=A.MoGeDepthEstimator(device=device),
        mesher=A.SAM3DObjectMesher(device=device),
        flow=_load_flow_model(device),       # SAM 3D flow backbone
        point_tracker=None,                  # plug in BootsTAPIR (adaptive guidance)
        gravity=None,                        # plug in GeoCalib (gravity alignment)
    )


def _load_flow_model(device: str):
    raise NotImplementedError(
        "Provide a FlowVelocityModel wrapping SAM 3D's flow backbone "
        "(see reconstruction/adapters.py:FlowVelocityModel and "
        "reconstruction/guided_tracking.py). It needs velocity(), "
        "sample_noise(), decode_pose()."
    )


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--video", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--device", default="cuda")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--contact-threshold", type=float, default=0.05)
    args = p.parse_args(argv)

    pipe = build_pipeline(args.device)
    pipe.cfg.contact_threshold = args.contact_threshold
    traj = pipe.run(args.video, max_frames=args.max_frames, stride=args.stride)
    io.save_handobj(traj, args.out)
    print(f"wrote {args.out}: {traj.n_frames} frames, "
          f"{int(traj.contact.sum())} in-contact, {traj.duration:.2f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
