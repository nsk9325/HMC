# Migration Review

Audit of everything this project depends on, for moving to a new machine.
Written 2026-08-13 against the current laptop (battery no longer holds a contest run).

This is the **inventory and review**, not the step-by-step plan. Setup instructions live in
[notice.md](notice.md) §1; this document records what must be carried over and what is
currently wrong.

---

## 1. Reference environment

The machine everything was developed and tested on:

```
Ubuntu 22.04.5 LTS
ROS 2 Humble
Python 3.10.12
NVIDIA GeForce GTX 1660 Ti (Max-Q), driver 580.173.02, torch CUDA 13.0
```

**Ubuntu 22.04 / Python 3.10 is mandatory, not a preference.** Three packages load their
libraries from `*.cpython-310.pyc` via `marshal`. Any other Python minor version cannot
unmarshal them, and there is no source fallback — `lib/*.py` are mirrors that nothing imports.

### Python packages

| Package | Version | Source | Notes |
|---|---|---|---|
| `ultralytics` | 8.4.118 | pip `~/.local` | must match the training version — `best.pt` is YOLO26-seg |
| `torch` | 2.12.1+cu130 | pip `~/.local` | see the inconsistency warning below |
| `torchvision` | 0.28.0 | pip `~/.local` | |
| `opencv-python` | 4.11.0 | pip `~/.local` | |
| `numpy` | 1.26.4 | pip `~/.local` | |
| `pyserial` | 3.5 | pip `~/.local` | |
| `keyboard` | 0.13.5 | pip `~/.local` | `data_collection.py` only; needs root on Linux |
| `scipy` | 1.8.0 | **apt** | `path_planner_node`'s `CubicSpline` |
| `setuptools` | **79.0.1** | pip `~/.local` | pinned — see below |
| `packaging` | ≥ 24 (26.3 here) | pip `~/.local` | pinned — see below |

### The setuptools / packaging pin

Three constraints intersect, and only a narrow window satisfies all of them:

| Requirement | Constraint |
|---|---|
| `colcon-core` | `setuptools < 80` |
| `torch` | `setuptools >= 77.0.3` |
| `setuptools >= 71` | `packaging >= 24` |

**Valid window: `77.0.3 <= setuptools < 80`.** Installing ultralytics pulls setuptools to 84,
which breaks `colcon build` on `interfaces_pkg` with a traceback that names `packaging` and
looks unrelated to the pip install that caused it. Do **not** use the `setuptools==58.2.0`
pin in `install.sh` either — that breaks torch.

### ROS packages (apt)

```
ros-humble-cv-bridge  ros-humble-message-filters  ros-humble-rclpy
ros-humble-launch  ros-humble-launch-ros  ros-humble-tf2
ros-humble-visualization-msgs
```

---

## 2. What must be carried over

### Tracked in git — modified originals

| File | Change |
|---|---|
| `src/camera_perception_pkg/camera_perception_pkg/image_publisher_node.py` | `DATA_SOURCE` → `camera`, `TIMER` 0.03 → 0.05 |
| `src/camera_perception_pkg/setup.py` | +2 console_scripts entry points |
| `src/decision_making_pkg/setup.py` | +1 console_scripts entry point |

`serial_sender_node.py` is **unmodified** — `/dev/ttyACM0` was correct on this machine.
`driving.ino`, `check_variable_resistor.ino` and `data_collection.py` carry changes that
pre-date this work (potentiometer calibration 407/243, baud 115200, `CAMERA_NUM = 2`).

### New source files

```
src/camera_perception_pkg/camera_perception_pkg/lane_extractor_node.py
src/camera_perception_pkg/camera_perception_pkg/traffic_light_state_node.py
src/decision_making_pkg/decision_making_pkg/mission_planner_node.py
src/launch_pkg/launch/mission.launch.py
project.md   notice.md   changes.md   migration.md
```

### Not in git — must be copied manually

| Item | Size | Required? |
|---|---|---|
| `best.pt` | 23 MB | **yes — nothing runs without it** |
| `img/` | 3 MB | **yes** — calibration frames; needed to re-derive `src_mat` if the camera is remounted |
| `Dataset/` | 56 MB | only to retrain |
| `debug_logs/` | 132 KB | disposable |

⚠️ `.gitignore` only excludes `/install /build /log`. A `git add .` would commit `best.pt`,
the 56 MB dataset, and every `__pycache__`. Extend it before pushing.

---

## 3. State that lives outside any file

These are invisible to git and each one fails silently.

**`dialout` group membership.** `/dev/ttyACM*` is `root:dialout`, mode `crw-rw----`.
`serial_sender_node` opens the port at *import*, so without group membership the node dies
during launch startup while every other node keeps running — `mission_planner` reports
`[DRIVING]` and publishes commands nobody receives. The car simply never moves.

```bash
sudo usermod -aG dialout $USER      # then log out and back in
```

**Clone location must be `~/ros2_ws`.** The `.pyc` loaders rebuild an absolute path from a
fixed slice of the current path and assume the workspace sits two levels below `/`.
`/home/user/dev/ros2_ws` resolves to `/home/user/dev/src/src/src/lib/...` and fails.
Any username and any workspace name are fine; an extra directory level is not.

**Device numbering.** On this machine the cameras enumerate as `/dev/video2` and
`/dev/video3`. Re-check on the new machine — see the open problem below.

---

## 4. Problems found during the audit

### 4.1 `CAM_NUM` does not match the actual device

`image_publisher_node.py` still has `CAM_NUM = 0`, but the only cameras present are
`/dev/video2` and `/dev/video3`. `data_collection.py` uses `CAMERA_NUM = 2`.

Whether this worked by luck of enumeration order is unclear, but it is not portable.
Check `ls /dev/video*` on the new machine and set `CAM_NUM` explicitly.

### 4.2 `CAP_PROP_BUFFERSIZE` was never applied

The camera branch of `image_publisher_node` sets width and height but not buffer depth, so
the V4L2 driver queues frames and `cap.read()` returns the oldest. Latency accumulates to a
constant lag and the controller steers on stale images — which contributes to the wobble
independently of any gain tuning. The course's own `data_collection` tool sets this; the ROS
node does not.

```python
self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # after the FRAME_WIDTH/HEIGHT calls
```

### 4.3 Inconsistent torch installation

```
torch-2.12.1.dist-info  AND  torch-2.13.0.dist-info
torchvision-0.27.1.dist-info  AND  torchvision-0.28.0.dist-info
torch.__version__ -> 2.12.1        pip list -> 2.13.0
```

A partial upgrade left two metadata sets. It functions, but do not reproduce it — install
cleanly on the new machine and verify `torch.__version__` matches what pip reports.

### 4.4 Mixed package sources

`scipy` comes from apt while `cv2`, `numpy` and `torch` come from pip in `~/.local`. This
happened by accident. Decide deliberately on the new machine so the set is reproducible.

---

## 5. Verification after migration

In order — each step gates the next.

```bash
# 1. environment
python3 --version                                    # 3.10.x
python3 -c "import setuptools, packaging; print(setuptools.__version__, packaging.__version__)"
id -nG | grep dialout
ls /dev/video* /dev/ttyACM*

# 2. model
cd ~/ros2_ws
python3 -c "from ultralytics import YOLO; m=YOLO('best.pt'); print(m.task, m.names)"
# expect: segment {0:'Car', 1:'Crosswalk', 2:'Greenlight', 3:'Lane1',
#                  4:'Lane2', 5:'Mid', 6:'Orange_light', 7:'Red_light', 8:'Stopline'}

# 3. build
source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash
# expect: 7 packages finished

# 4. perception only — car cannot move
ros2 launch launch_pkg mission.launch.py data_source:=camera
ros2 topic hz /detections                            # >= 10 Hz
ros2 topic echo /yolov8_lane_info --once             # target_x must vary, not sit at 128

# 5. serial reaches the Arduino
ros2 launch launch_pkg mission.launch.py data_source:=camera use_serial:=true
ros2 node list | grep serial                         # serial_sender_node must be present
```

Then re-do the steering-sign hand-push check from [notice.md](notice.md) §4.1 before the
first driving run. The camera mount is the one thing that could shift during a physical move,
and it invalidates `src_mat`.

---

## 6. Calibration values — carried in code, verify on the new machine

| Parameter | Value | Where | Sensitive to |
|---|---|---|---|
| `src_mat` | `[258,240, 452,240, 547,380, 170,380]` | `lane_extractor_node.py` | camera position/angle |
| `lane_width` | 256 | `lane_extractor_node.py` | fixed by `dst_mat` geometry |
| `car_center_point` | `[294, 179]` | `mission.launch.py` | camera lateral alignment |
| `steering_gain` / `lookahead_near` / `far` / `far_weight` | 0.30 / 30 / 85 / 0.8 | `mission_planner_node.py` | vehicle, not machine |
| `car_min_bbox_height` | 165 | `mission_planner_node.py` | camera position |
| `resistance_most_left/right` | 407 / 243 | `driving.ino` | this specific vehicle |

None depend on the laptop. All depend on the camera and the car — so they survive a machine
change but not a remount. `img/*.png` is what lets you re-derive `src_mat`, which is why it
must be carried over.

---

## 7. Still outstanding (independent of the migration)

- `creep_duration_sec` (0.8) unmeasured — the stop sequence has **never actually executed**
  in a real run; the run that reached `APPROACH` did so via the lap-counting bug and never
  saw the parked car.
- Lap counting fix (`crosswalk_clear_sec 2.0`, `lap_cooldown_sec 15.0`) validated by replay
  against the recorded log, but not yet on a live two-lap run.
- Lidar never tested; obstacle handling is dead code with the nodes commented out.
- `stop_on_red_while_driving` defaults to `False` — confirm the rules before the contest.
