"""End-to-end test of the embodiment-agnostic half of the pipeline.

Runs with only numpy/scipy installed (no ROS, no foundation models):

    python3 -m pytest test/test_pipeline.py        # or
    python3 test/test_pipeline.py                  # prints a summary

Validates the SE(3) math, the synthetic Stage-A stand-in, and Stage-B
retargeting, including the key invariant that object-anchored and wrist-driven
EE poses coincide at every grasp onset (so the path is continuous).
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hitbot_imitation.demo import make_pick_and_place          # noqa: E402
from hitbot_imitation.retargeting import RetargetConfig, retarget  # noqa: E402
from hitbot_imitation.retargeting.grasp import detect_segments     # noqa: E402
from hitbot_imitation.utils import io, se3                         # noqa: E402


def test_se3_roundtrip():
    t = np.array([0.1, -0.2, 0.3])
    Rm = se3.R.from_euler("xyz", [0.3, -0.5, 1.1]).as_matrix()
    T = se3.make_pose(t, Rm)
    assert np.allclose(se3.compose(T, se3.inverse(T)), np.eye(4), atol=1e-9)
    assert np.allclose(se3.translation(T), t)


def test_interpolate_endpoints():
    traj = make_pick_and_place(n=60)
    src = traj.timestamps - traj.timestamps[0]
    dst = np.linspace(0, src[-1], 200)
    out = se3.interpolate_poses(traj.object_poses, src, dst)
    assert np.allclose(out[0], traj.object_poses[0], atol=1e-6)
    assert np.allclose(out[-1], traj.object_poses[-1], atol=1e-6)


def test_grasp_segments():
    traj = make_pick_and_place(n=120)
    segs = detect_segments(traj.contact)
    assert len(segs) == 1
    assert segs[0].start == 30 and segs[0].end == 89   # n//4 .. 3n//4-1


def test_retarget_continuity():
    """Object-anchored and wrist-driven EE must coincide at grasp onset."""
    traj = make_pick_and_place(n=120)
    cfg = RetargetConfig()
    segs = detect_segments(traj.contact, cfg.min_grasp_len)
    seg = segs[0]
    from hitbot_imitation.retargeting.retarget import _grip_transform
    T_ee_obj = _grip_transform(traj.hand_poses[seg.start],
                               traj.object_poses[seg.start], cfg.hand_to_ee)
    ee_from_obj = traj.object_poses[seg.start] @ se3.inverse(T_ee_obj)
    ee_from_wrist = traj.hand_poses[seg.start] @ cfg.hand_to_ee
    assert np.allclose(ee_from_obj, ee_from_wrist, atol=1e-9)


def test_retarget_shapes_and_io(tmp_path=None):
    traj = make_pick_and_place(n=120, fps=30.0)
    cfg = RetargetConfig(control_rate=50.0)
    ee = retarget(traj, cfg)
    # resampled to 50 Hz over the same duration
    expected = int(np.floor(traj.duration * 50.0)) + 1
    assert abs(ee.n_frames - expected) <= 1
    assert ee.ee_poses.shape == (ee.n_frames, 4, 4)
    assert set(np.unique(ee.gripper)).issubset({0.0, 1.0})
    assert ee.gripper.sum() > 0           # something was grasped
    assert ee.ee_link == "S922_7"

    # round-trip through disk
    import tempfile
    d = tmp_path or tempfile.mkdtemp()
    p = os.path.join(str(d), "ee.npz")
    io.save_ee(ee, p)
    ee2 = io.load_ee(p)
    assert np.allclose(ee.ee_poses, ee2.ee_poses)
    return ee


if __name__ == "__main__":
    test_se3_roundtrip()
    test_interpolate_endpoints()
    test_grasp_segments()
    test_retarget_continuity()
    ee = test_retarget_shapes_and_io()
    print("ALL TESTS PASSED")
    print(f"  EE frames        : {ee.n_frames} @ 50 Hz")
    print(f"  grasp frames      : {int(ee.gripper.sum())} closed")
    print(f"  reach report      : {ee.meta['reach']}")
    print(f"  base/ee frame     : {ee.base_frame} -> {ee.ee_link}")
    print(f"  start EE xyz       : {se3.translation(ee.ee_poses[0]).round(3)}")
    print(f"  mid   EE xyz       : {se3.translation(ee.ee_poses[ee.n_frames//2]).round(3)}")
    print(f"  end   EE xyz       : {se3.translation(ee.ee_poses[-1]).round(3)}")
