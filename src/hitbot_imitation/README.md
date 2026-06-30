# hitbot_imitation

**"Do as I Do" — learning manipulation from everyday human videos, adapted to the
HITBOT Z-Arm S922 (6-DOF) and executed in Gazebo via MoveIt.**

This package reimplements the pipeline of *Do as I Do: Dexterous Manipulation
Data from Everyday Human Videos* (Paliwal et al.), adapted for a **6-DOF arm
with no dexterous hand**. It reconstructs a 4D hand-object trajectory from a
monocular RGB video and retargets it onto the arm's tool flange.

---

## The embodiment adaptation (read this first)

The original paper retargets human **finger** articulation onto a 22-DoF
dexterous hand. The S922 has **no fingers**, so we cannot reproduce dexterous
in-hand manipulation. Instead we retarget at the level the arm *can* reproduce:

| Paper (dexterous hand) | Here (6-DOF S922) |
| --- | --- |
| 22-DoF finger articulation | Tool-flange 6-DOF pose (`S922_7`) |
| Per-finger contact forces | 1-DoF gripper open/close (if a gripper is fitted) |
| MuJoCo-Warp dynamics-aware finger retargeting | Object/wrist pose tracking + MoveIt IK |
| Sharpa Wave hand + UR3e | HITBOT Z-Arm S922 |

Concretely, the human **wrist** is treated as the analogue of the **tool
flange**, and the grasped **object's** 6-DOF path is reproduced while it is held
(so pour / stir / place motions are faithful). Fingers collapse into a gripper
signal. This is the honest, runnable subset of the paper for this hardware.

> If you later mount a parallel gripper or a multi-fingered hand, the contact
> signal and grip transform are already plumbed through (`retargeting/grasp.py`,
> `RetargetConfig.hand_to_ee`).

---

## Architecture — three decoupled stages

```
 video.mp4
    │  Stage A  reconstruction   (GPU machine: HaWoR, SAM3, MoGe, SAM3D, ...)
    ▼
 handobj_traj.npz      ← HandObjectTrajectory (object+hand poses, contact)
    │  Stage B  retargeting      (pure numpy/scipy — runs anywhere)
    ▼
 ee_traj.npz           ← EETrajectory (tool-flange poses + gripper)
    │  Stage C  execution        (ROS 2 + MoveIt + Gazebo)
    ▼
 arm moves in Gazebo
```

The stages are decoupled on disk (`.npz`), so you can run the heavy Stage A on
the GPU laptop and iterate on Stage B/C anywhere.

### Code map
```
hitbot_imitation/
  types.py                     HandObjectTrajectory, EETrajectory
  utils/se3.py                 SE(3) math, SLERP resample, gravity align
  utils/io.py                  .npz load/save
  reconstruction/              STAGE A
    adapters.py                model interfaces + lazy backend wrappers
    guided_tracking.py         guided-diffusion blending (Eq.1), adaptive
                               guidance (App.A), SE(3) clustering select (Eq.3)
    alignment.py               hand-object + gravity alignment (App.A)
    pipeline.py                orchestration: video -> HandObjectTrajectory
  retargeting/                 STAGE B
    grasp.py                   contact -> grasp segments + gripper signal
    workspace.py               place motion into the robot workspace
    retarget.py                object/wrist pose -> EETrajectory
  demo/synthetic.py            synthetic pick-and-place (no models needed)
  app_reconstruct.py / app_retarget.py / app_execute.py   console entry points
launch/imitation.launch.py     Gazebo + MoveIt bring-up
config/retarget.yaml           tunable retargeting parameters
test/test_pipeline.py          end-to-end test (numpy/scipy only)
```

---

## Quick start (no foundation models needed)

Stage B + C work immediately with a synthetic trajectory — good for bringing up
Gazebo execution end to end before any perception models are installed.

```bash
# 0) build
cd hitbot_ws
colcon build --packages-select hitbot_description hitbot_moveit_config hitbot_imitation
source install/setup.bash

# 1) Stage B: synthesise a pick-and-place and retarget it to the arm
ros2 run hitbot_imitation retarget --demo --out /tmp/ee_traj.npz
#   (or: python3 src/hitbot_imitation/scripts/retarget.py --demo --out /tmp/ee_traj.npz)

# 2) bring up the robot in Gazebo + MoveIt
ros2 launch hitbot_imitation imitation.launch.py gui:=true

# 3) in a second terminal: execute the trajectory on the arm
source install/setup.bash
ros2 run hitbot_imitation execute_moveit --ros-args \
    -p trajectory:=/tmp/ee_traj.npz -p dry_run:=false
```

`execute_moveit` solves IK per waypoint via MoveIt `/compute_ik` (seeded for
continuity), streams the result through `arm_controller`, and publishes the
reconstructed EE path as an RViz `Marker` (`imitation_ee_path`).

Use `-p dry_run:=true` to validate IK/reachability without moving the arm.

---

## Stage A — reconstruction (GPU machine)

Stage A needs the perception foundation models. They are **not** required to
build the package or to run the demo above.

1. Install the backends (see `requirements.txt` for repo links):
   HaWoR (hands), SAM 3 (segmentation), MoGe (metric depth), SAM 3D (object
   mesh + flow backbone); optionally BootsTAPIR (adaptive guidance) and GeoCalib
   (gravity).
2. Wire each adapter in `reconstruction/adapters.py` to the installed API, and
   provide a `FlowVelocityModel` + `build_pipeline()` in `app_reconstruct.py`.
3. Run:
   ```bash
   ros2 run hitbot_imitation reconstruct \
       --video clip.mp4 --out handobj_traj.npz --max-frames 300
   ```
   The paper's *algorithms* (guided-diffusion pose blending, adaptive guidance,
   SE(3) clustering selection, hand-object/gravity alignment) are already
   implemented in `guided_tracking.py` / `alignment.py`; you only supply the
   model inference via the adapters.

Then feed the result into Stage B:
```bash
ros2 run hitbot_imitation retarget --in handobj_traj.npz --out ee_traj.npz \
    --anchor-xyz 0.35 0.0 0.30 --anchor-yaw 0.0 --scale 1.0
```

---

## Tuning (config/retarget.yaml or CLI flags)

- `--anchor-xyz / --anchor-yaw` — where in the robot base frame the motion starts
  (the paper's manual workspace alignment).
- `--scale` — shrink a large human motion into the arm's reach (`<1`).
- `--no-object-anchor` — track the wrist throughout instead of the held object.
- `control_rate` — output rate (default 50 Hz, matching the paper).

`retarget` prints a reachability report; if waypoints exceed the ~0.92 m S922
envelope, lower `--scale` or move the anchor closer to the base.

---

## Robot model note

`hitbot.urdf` (Gazebo) and `hitbot_real.urdf` (MoveIt + real hardware) now share
**identical kinematics** and the updated ZARM `S922-*.stl` meshes; they differ
only in the `ros2_control` hardware (`gazebo_ros2_control` vs the TCP driver).
This guarantees that what MoveIt plans matches what Gazebo simulates and what the
real arm executes.

To run on the **real** robot instead of Gazebo, bring up
`hitbot_moveit_config/launch/moveit_real.launch.py` and run the same
`execute_moveit` command — Stage B/C are identical.

---

## Limitations (inherited + embodiment)

- No dexterous in-hand manipulation (no fingers) — only tool-flange/object pose.
- Reconstruction assumes rigid objects and reasonable monocular metric depth.
- Monocular contact is ambiguous; the `contact_threshold` may need per-clip tuning.
- The arm tracks the object/wrist pose; it does not reason about the scene
  (obstacles, articulation) beyond MoveIt's own collision checking.
