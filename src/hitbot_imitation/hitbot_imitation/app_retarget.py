"""Stage B entry point: HandObjectTrajectory (.npz) -> EETrajectory (.npz).

Pure NumPy/SciPy.  Exposed as the ``retarget`` console script and runnable via
``scripts/retarget.py``.
"""
from __future__ import annotations

import argparse

from .demo import make_pick_and_place
from .retargeting import RetargetConfig, WorkspaceConfig, retarget
from .utils import io, se3


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--in", dest="inp", help="input HandObjectTrajectory .npz")
    src.add_argument("--demo", action="store_true",
                     help="synthesise a pick-and-place instead of loading a file")
    p.add_argument("--out", required=True, help="output EETrajectory .npz")
    p.add_argument("--rate", type=float, default=50.0, help="control rate (Hz)")
    p.add_argument("--anchor-xyz", type=float, nargs=3, default=[0.35, 0.0, 0.30],
                   metavar=("X", "Y", "Z"), help="robot-frame anchor for start")
    p.add_argument("--anchor-yaw", type=float, default=0.0, help="yaw (rad)")
    p.add_argument("--scale", type=float, default=1.0, help="motion scale")
    p.add_argument("--smooth", type=int, default=7, help="smoothing window")
    p.add_argument("--no-object-anchor", action="store_true",
                   help="track the wrist throughout instead of the object")
    args = p.parse_args(argv)

    traj = (make_pick_and_place() if args.demo else io.load_handobj(args.inp))

    cfg = RetargetConfig(
        control_rate=args.rate,
        smooth_window=args.smooth,
        object_anchored_grasp=not args.no_object_anchor,
        workspace=WorkspaceConfig(anchor_xyz=tuple(args.anchor_xyz),
                                  anchor_yaw=args.anchor_yaw, scale=args.scale),
    )
    ee = retarget(traj, cfg)
    io.save_ee(ee, args.out)

    reach = ee.meta["reach"]
    print(f"wrote {args.out}")
    print(f"  frames        : {ee.n_frames} @ {args.rate:g} Hz "
          f"({ee.timestamps[-1]:.2f}s)")
    print(f"  grasp segments : {ee.meta['n_segments']}, "
          f"closed on {int(ee.gripper.sum())} frames")
    print(f"  reach          : dist [{reach['min_dist']:.2f}, "
          f"{reach['max_dist']:.2f}] m, out-of-reach {reach['n_out_of_reach']}")
    print(f"  start EE xyz    : {se3.translation(ee.ee_poses[0]).round(3)}")
    print(f"  end   EE xyz    : {se3.translation(ee.ee_poses[-1]).round(3)}")
    if reach["n_out_of_reach"]:
        print("  ! some waypoints exceed the reach envelope — lower --scale "
              "or move --anchor-xyz closer to the base.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
