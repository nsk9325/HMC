# Migration

Everything needed to bring this project up on a new machine, followed by the audit it was
derived from.

Target machine on this migration: repo cloned as **`~/HMC`**.

Related: [notice.md](notice.md) (operational reference), [project.md](project.md) (what the
project is), [changes.md](changes.md) (what was changed and why).

---

# PART 1 — Do this, in order

Each step gates the next. Do not skip ahead; several failures downstream are silent.

## Step 0 — Check where you cloned it

```bash
cd ~/HMC && pwd
```

Must be **exactly two levels below `/`** — `/home/<user>/<name>`. The name is irrelevant
(`HMC`, `ros2_ws`, anything). An extra level (`~/dev/HMC`, `~/HMC/workspace`) breaks the
`.pyc` library loaders with `FileNotFoundError`. If it's nested, move it now.

**Always run commands from the workspace root** — `best.pt` and the bundled video path
resolve against the working directory.

## Step 1 — System packages

Requires **Ubuntu 22.04 + ROS 2 Humble + Python 3.10**. Not negotiable: the course libraries
ship only as `*.cpython-310.pyc` and are loaded via `marshal`.

```bash
sudo apt update
sudo apt install -y python3-pip python3-scipy \
    ros-humble-cv-bridge ros-humble-message-filters \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-visualization-msgs
```

## Step 2 — Python packages

⚠️ **The last line must come last.** Installing ultralytics drags setuptools to 84, which
breaks `colcon build`.

```bash
pip install ultralytics==8.4.118 opencv-python==4.11.0 pyserial==3.5 keyboard==0.13.5
pip install "setuptools==79.0.1" "packaging>=24.0"
```

Verify:

```bash
python3 -c "import setuptools, packaging, torch; \
print(setuptools.__version__, packaging.__version__, torch.__version__, torch.cuda.is_available())"
# setuptools 79.0.1 · packaging >= 24 · cuda True
```

If `cuda` is `False`: `sudo ubuntu-drivers autoinstall` then reboot. CPU inference runs at
~3 Hz — too slow to drive.

## Step 3 — Serial permissions

```bash
sudo usermod -aG dialout $USER
```

**Log out and back in.** Group changes only apply to new login sessions.

```bash
id -nG | grep dialout
```

Skip this and the system looks completely healthy while the car never moves — see §D.1.

## Step 4 — Find the devices

```bash
ls /dev/video* /dev/ttyACM* /dev/ttyUSB*
```

Write down what you actually see. Do not assume the old numbering.

## Step 5 — Apply the two outstanding code fixes

Both are in `src/camera_perception_pkg/camera_perception_pkg/image_publisher_node.py`.

**5a. Camera number** — line 24, currently `CAM_NUM = 0`. Set it to the number from step 4.
On the old machine the cameras enumerated as `/dev/video2` and `/dev/video3`.

**5b. Frame buffering** — in the `data_source == 'camera'` branch, after the two `set` calls:

```python
self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
```

Without it the V4L2 driver queues frames and `cap.read()` returns the oldest, so the
controller steers on stale images. This has never been applied and is a live contributor to
the observed wobble.

If the Arduino is not `ttyACM0`, also edit `PORT` at
`src/serial_communication_pkg/serial_communication_pkg/serial_sender_node.py:17`. That one has
no parameter override.

## Step 6 — Verify the model

```bash
cd ~/HMC
ls -la best.pt          # 23 MB
python3 -c "from ultralytics import YOLO; m=YOLO('best.pt'); print(m.task, m.names)"
```

Expected:

```
segment {0:'Car', 1:'Crosswalk', 2:'Greenlight', 3:'Lane1', 4:'Lane2',
         5:'Mid', 6:'Orange_light', 7:'Red_light', 8:'Stopline'}
```

`task` must be `segment` — the lane pipeline consumes masks.

## Step 7 — Build

```bash
cd ~/HMC
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

Expect **7 packages finished**. `UserWarning: Unknown distribution option: 'tests_require'`
is harmless — it comes from the course's own `setup.py` files.

If `interfaces_pkg` fails with `canonicalize_version() got an unexpected keyword argument`,
the setuptools pin didn't take. Redo the last line of step 2.

## Step 8 — Perception only (car cannot move)

```bash
ros2 launch launch_pkg mission.launch.py data_source:=camera
```

Second terminal:

```bash
cd ~/HMC && source install/setup.bash
ros2 topic hz /detections                    # >= 10 Hz
ros2 topic echo /yolov8_lane_info --once     # target_x must VARY, not sit at 128
```

`target_x` stuck at 128 means no lane found (128 = `lane_width/2`, the silent fallback).

Watch the `roi_img` window: on a straight, two roughly **vertical, parallel** edges. If they
lean or converge, the camera moved in transit and `src_mat` needs re-deriving — see §F.

## Step 9 — Steering sign, by hand

Motors off, node running, push the car by hand.

```bash
ros2 topic pub -r 10 /yolov8_traffic_light_info std_msgs/String "data: 'Green'"
ros2 topic echo /topic_control_signal
```

| Car position | Expected `steering` |
|---|---|
| left of lane centre | **positive** |
| right of lane centre | **negative** |
| centred on a straight | near zero |

Do not skip. An inverted sign puts the car in the grass within a metre and is
indistinguishable from a calibration or gain fault.

## Step 10 — Serial reaches the Arduino

```bash
ros2 launch launch_pkg mission.launch.py data_source:=camera use_serial:=true
ros2 node list | grep serial      # serial_sender_node MUST be listed
```

If it's missing, the node died at startup — almost always step 3.

## Step 11 — First drive

Wheels off the ground first, then on.

```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false wait_for_green:=false debug_log:=true
```

```bash
ros2 param set /mission_planner_node cruise_speed 80
```

Work up toward 175 only once it holds a lane at 120.

## Command reference

```bash
cd ~/HMC && source install/setup.bash

# perception only, cannot move
ros2 launch launch_pkg mission.launch.py data_source:=camera

# tuning: no light gate, logging on
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false wait_for_green:=false debug_log:=true

# real mission run
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false
```

Ctrl-C sends `s0l0r0` and stops the car. Keep the power switch as the real backup.

---

# PART 2 — Reference

## A. Environment this was developed against

```
Ubuntu 22.04.5 LTS · ROS 2 Humble · Python 3.10.12
NVIDIA GeForce GTX 1660 Ti (Max-Q), driver 580.173.02, torch CUDA 13.0
```

| Package | Version | Source | Notes |
|---|---|---|---|
| `ultralytics` | 8.4.118 | pip | must match training version — `best.pt` is YOLO26-seg |
| `torch` | 2.12.1+cu130 | pip | see §D.3 |
| `torchvision` | 0.28.0 | pip | |
| `opencv-python` | 4.11.0 | pip | |
| `numpy` | 1.26.4 | pip | |
| `pyserial` | 3.5 | pip | |
| `keyboard` | 0.13.5 | pip | `data_collection.py` only; needs root on Linux |
| `scipy` | 1.8.0 | apt | `path_planner_node`'s `CubicSpline` |
| `setuptools` | **79.0.1** | pip | pinned, see §B |
| `packaging` | ≥ 24 | pip | pinned, see §B |

## B. The setuptools / packaging pin

| Requirement | Constraint |
|---|---|
| `colcon-core` | `setuptools < 80` |
| `torch` | `setuptools >= 77.0.3` |
| `setuptools >= 71` | `packaging >= 24` |

**Valid window: `77.0.3 <= setuptools < 80`.** Do not use `install.sh`'s `setuptools==58.2.0`
either — that breaks torch. The failure mode is a `colcon` traceback naming `packaging` that
looks unrelated to the pip install that caused it, and only `interfaces_pkg` fails directly;
everything else aborts as a dependent.

## C. What the repo contains

**Modified originals**

| File | Change |
|---|---|
| `image_publisher_node.py` | `DATA_SOURCE` → `camera`, `TIMER` 0.03 → 0.05 |
| `camera_perception_pkg/setup.py` | +2 console_scripts entry points |
| `decision_making_pkg/setup.py` | +1 console_scripts entry point |

`serial_sender_node.py` is unmodified. `driving.ino`, `check_variable_resistor.ino` and
`data_collection.py` carry changes predating this work (potentiometer 407/243, baud 115200).

**New files**

```
src/camera_perception_pkg/camera_perception_pkg/lane_extractor_node.py
src/camera_perception_pkg/camera_perception_pkg/traffic_light_state_node.py
src/decision_making_pkg/decision_making_pkg/mission_planner_node.py
src/launch_pkg/launch/mission.launch.py
project.md   notice.md   changes.md   migration.md
```

**Data**

| Item | Size | In repo? |
|---|---|---|
| `best.pt` | 23 MB | yes — nothing runs without it |
| `img/` | 3 MB | yes — calibration frames, needed to re-derive `src_mat` |
| `Dataset/` | 56 MB | no — re-export from Roboflow (`autodrive-vmctm` v4) |
| `debug_logs/` | — | no — regenerated each run |

## D. Problems found in the audit

### D.1 `dialout` membership fails silently
`serial_sender_node` opens the port at *import*, so a permission error kills it during launch
startup while every other node keeps running. `mission_planner` reports `[DRIVING]` and
publishes commands nobody receives. Diagnosed once already; cost real debugging time.

### D.2 `CAM_NUM` did not match the device
Still `0` in the file while the machine only exposed `/dev/video2` and `/dev/video3`.
Handled in step 5a.

### D.3 Inconsistent torch install on the old machine
```
torch-2.12.1.dist-info AND torch-2.13.0.dist-info
torch.__version__ -> 2.12.1     pip list -> 2.13.0
```
A partial upgrade left two metadata sets. Install cleanly on the new machine and confirm the
two agree.

### D.4 Mixed package sources
`scipy` from apt; `cv2`, `numpy`, `torch` from pip. Accidental on the old machine, made
deliberate in step 1 / step 2.

## E. Calibration values — machine-independent

None of these depend on the laptop. All depend on the **camera mount and the vehicle**, so
they survive a machine change but not a remount.

| Parameter | Value | Where |
|---|---|---|
| `src_mat` | `[258,240, 452,240, 547,380, 170,380]` | `lane_extractor_node.py` |
| `lane_width` | 256 | `lane_extractor_node.py` |
| `car_center_point` | `[294, 179]` | `mission.launch.py` |
| `steering_gain` | 0.30 | `mission_planner_node.py` |
| `lookahead_near` / `far` / `far_weight` | 30 / 85 / 0.8 | `mission_planner_node.py` |
| `car_min_bbox_height` | 165 | `mission_planner_node.py` |
| `crosswalk_clear_sec` / `lap_cooldown_sec` | 2.0 / 15.0 | `mission_planner_node.py` |
| `resistance_most_left/right` | 407 / 243 | `driving.ino` (this vehicle only) |

## F. If the camera moved in transit

`src_mat` becomes wrong and the car will hold a lateral offset or weave. Re-derive it from
three photos on a straight section — that is what `img/*.png` is for. Method is recorded in
[notice.md](notice.md) §4.2: lane width is linear in image row on a flat plane, so fit the
`Lane2` mask boundaries over `y = 230…380` and read them at `y = 240` and `y = 380`.

## G. Still outstanding, independent of the migration

- `creep_duration_sec` (0.8) unmeasured — the stop sequence has **never actually executed**.
  The one run that reached `APPROACH` got there via the lap-counting bug and never saw the
  parked car.
- Lap-counting fix validated by replaying the recorded log, but not yet on a live two-lap run.
- Lidar never tested; obstacle handling is dead code with the nodes commented out.
- `stop_on_red_while_driving` defaults to `False` — confirm against the contest rules.
