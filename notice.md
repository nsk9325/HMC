# NOTICE — read this before running anything

Operational checklist for this project: what must be installed, what must be calibrated,
what silently fails, and how to bring the system up on a fresh laptop.

Companion documents:
- [project.md](project.md) — what the project *is* (mission, track, labels, hardware)
- [changes.md](changes.md) — what was changed and why

---

## 1. Fresh laptop / fresh clone — setup order

### ⚠️ 1.1 The repository must sit exactly two levels below `/`

Any name works (`~/ros2_ws`, `~/HMC`, …) and any username works. What breaks is **extra
nesting**.

This is not a style preference. `lib/__init__.py` in three packages loads its compiled
`.pyc` library by **reconstructing an absolute path from a hardcoded slice of the current
path**:

```python
p = os.path.dirname(os.path.abspath(__file__)).split("/")
LIB_PATH = os.path.join("/", *p[1:4], "src", *p[5:6], *p[5:6], "lib", file_name)
```

It assumes the workspace root sits exactly two levels below `/`. Verified behaviour:

| Clone location | Resolved library path | Result |
|---|---|---|
| `/home/user/ros2_ws` | `/home/user/ros2_ws/src/.../lib/*.pyc` | ✅ works |
| `/home/student/ros2_ws` | `/home/student/ros2_ws/src/.../lib/*.pyc` | ✅ works (any username) |
| `/home/user/HMC` | `/home/user/HMC/src/.../lib/*.pyc` | ✅ works (any workspace name) |
| **`/home/user/dev/HMC`** | **`/home/user/dev/src/src/src/lib/*.pyc`** | ❌ `FileNotFoundError` |
| **`/home/user/HMC/workspace`** | **`/home/user/HMC/src/src/src/lib/*.pyc`** | ❌ `FileNotFoundError` |

Any extra directory level breaks it. Clone to `~/<name>` directly — never nested.
Nothing in the source hardcodes the workspace name; all paths are relative or `__file__`-based.
But **always run commands from the workspace root**, since `best.pt` and the bundled video
path resolve against the current working directory.

### 1.2 Python must be 3.10

The libraries ship as `*.cpython-310.pyc` and are loaded via `marshal`. A different Python
minor version cannot unmarshal them. ROS 2 Humble on Ubuntu 22.04 gives 3.10 by default —
do not use a newer distro or a conda env with a different Python.

### 1.3 Install

```bash
sudo apt update && sudo apt install -y python3-pip
pip install ultralytics==8.4.118 opencv-python pyserial
pip install "setuptools==79.0.1" "packaging>=24.0"     # ← 순서 중요, 아래 참조
```

⚠️ **Do not use the `setuptools==58.2.0` pin from `install.sh`.** Three packages impose
conflicting constraints, and only a narrow window satisfies all of them:

| Package | Requires | If violated |
|---|---|---|
| `colcon-core` | `setuptools < 80` | — |
| `torch` | `setuptools >= 77.0.3` | — |
| `setuptools >= 71` | `packaging >= 24` | build failure below |

**Valid window: `77.0.3 <= setuptools < 80`.** `79.0.1` is known good.

Installing ultralytics pulls setuptools up to 84, which breaks `colcon build` on
`interfaces_pkg` with a misleading traceback:

```
TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
...
Failed <<< interfaces_pkg
```

The cause is setuptools ≥ 71 calling a `packaging` API that Ubuntu's apt-supplied
`packaging 21.3` does not have. Only `interfaces_pkg` fails (it uses `ament_cmake_python`);
the pure-Python packages build fine, so the error looks unrelated to the pip install that
caused it. Every other package then aborts because they depend on `interfaces_pkg`.

Verify after installing:

```bash
python3 -c "import setuptools, packaging; print(setuptools.__version__, packaging.__version__)"
# expect: 79.0.1 26.3  (or any setuptools in [77.0.3, 80) and packaging >= 24)
```

Harmless leftover: every ament_python package prints
`UserWarning: Unknown distribution option: 'tests_require'` during the build. That comes from
the course's own `setup.py` files and can be ignored.

**Pin ultralytics to 8.4.118.** `best.pt` is a YOLO26-seg model using `C3k2` / `C2PSA`
blocks. Older builds fail with:

```
AttributeError: Can't get attribute 'C3k2' on <module 'ultralytics.nn.modules.block'>
```

`yolov8_node` catches that, logs "Error while loading model", returns `FAILURE` — and then
publishes **nothing at all**, with every downstream node idle and no obvious cause.
`install.sh` installs ultralytics unpinned, so do not rely on it.

### 1.4 Model weights

Put `best.pt` at the **workspace root** (`~/ros2_ws/best.pt`). The path resolves against the
working directory, so it must be there when you run `ros2 launch` from the workspace root.

Verify before anything else:

```bash
python3 -c "from ultralytics import YOLO; m=YOLO('best.pt'); print(m.task, m.names)"
# expect: segment {0:'Car', 1:'Crosswalk', 2:'Greenlight', 3:'Lane1', 4:'Lane2',
#                  5:'Mid', 6:'Orange_light', 7:'Red_light', 8:'Stopline'}
```

If `task` is not `segment`, the lane pipeline cannot work — masks are required.

### 1.5 Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### 1.6 Device names

```bash
ls /dev/ttyACM*    # Arduino  (expected ttyACM0)
ls /dev/video*     # camera   (expected video0)
ls /dev/ttyUSB*    # lidar    (expected ttyUSB0)
```

⚠️ The serial port in `serial_sender_node.py` is a **module-level constant, not a
parameter**, and the port is opened at *import* time. If the Arduino enumerates as
`ttyACM1`, you must edit the file — no `--ros-args` override exists. The node also fails
instantly if the Arduino is absent, which takes the whole launch file down with it.

Device names change if you unplug hardware while a node is running. Stop the nodes first.

### ⚠️ 1.7 Serial permissions — the user must be in `dialout`

`/dev/ttyACM*` is owned `root:dialout` with mode `crw-rw----`, so a user outside that group
cannot open it at all.

```bash
sudo usermod -aG dialout $USER
# then LOG OUT AND BACK IN — group changes only apply to new login sessions
# (or `newgrp dialout` for the current shell only)
```

Verify:

```bash
id -nG | grep dialout
python3 -c "import os; print(os.access('/dev/ttyACM0', os.W_OK))"   # must be True
```

**Why this is worth its own section**: `serial_sender_node` opens the port at *import* time,
so a permission error kills the node during launch startup. `ros2 launch` keeps every other
node alive, so the system looks healthy — `mission_planner` reports `[DRIVING]` and publishes
steering and speed commands to a topic with no subscriber. The car simply never moves, with
no error visible unless you scroll back through the launch output.

Quick check that it survived:

```bash
ros2 node list | grep serial     # serial_sender_node must be present
```

This also affects `data_collection.py`, which opens the same port.

---

## 2. Which nodes to run

Replacement nodes were added as new files; the course originals are untouched and still
registered. **Use `mission.launch.py`, not `main.launch.py`.**

| Use this | Not this | Why |
|---|---|---|
| `lane_extractor_node` | `lane_info_extractor_node` | original hardcodes `'lane2'`; dataset has `Lane2` |
| `traffic_light_state_node` | `traffic_light_detector_node` | original matches `'traffic_light'`, a class that no longer exists |
| `mission_planner_node` | `motion_planner_node` | original has no mission state machine |

```bash
ros2 launch launch_pkg mission.launch.py
```

For bench testing, comment out `serial_sender_node` in the launch file first.

---

## 3. Parameters to change

### 3.1 Must change before the first run

| Node | Parameter | Current | Set to | Why |
|---|---|---|---|---|
| `yolov8_node` | `device` | `cpu` | `cuda:0` | GTX 1660 available; CPU cannot keep up |
| `yolov8_node` | `threshold` | `0.5` | `0.3` | `Lane2` confidence dips to 0.37–0.69; at 0.5 the lane drops out |
| `image_publisher_node` | `data_source` | `video` | `camera` | for real driving (leave as `video` for bench tests) |
| `mission_planner_node` | `cruise_speed` | `175` | `80` for first drives | raise only once steering is tuned |

### 3.2 Must be calibrated on the real vehicle — see §4

| Node | Parameter | Current | Status |
|---|---|---|---|
| `lane_extractor_node` | `src_mat` | `[258,240, 452,240, 547,380, 170,380]` | ✅ **calibrated** 2026-08-13 from `img/*.png` — see §4.2 |
| `lane_extractor_node` | `lane_width` | `256` | ✅ **calibrated** (measured 257, theory 256) |
| `path_planner_node` | `car_center_point` | `[294, 179]` | ⚠️ **calibrated but assumes the camera sits on the vehicle centreline** — verify on the car |
| `mission_planner_node` | `steering_gain` | `0.35` | **guess** |
| `mission_planner_node` | `lookahead_index` | `40` | **guess** |
| `mission_planner_node` | `car_min_bbox_height` | `120` | **guess** — measurable, see §4.3 |
| `mission_planner_node` | `stopline_trigger_y` | `430` | **guess** — measurable, see §4.3 |
| `mission_planner_node` | `creep_duration_sec` | `0.8` | **guess** — trial and error only |
| `mission_planner_node` | `crosswalk_trigger_y` | `300` | **guess** — validate over one lap |

### 3.3 Decisions to confirm, not measure

| Parameter | Current | Question |
|---|---|---|
| `stop_on_red_while_driving` | `False` | Must the car obey a red light on laps 1 and 2, or is the signal only a start gate? |
| `target_lap_count` | `2` | Does the start position sit before or after the crosswalk? Affects whether 2 crosswalk passes equals 2 laps. |
| `path_timeout_sec` | `1.0` | Measured worst dropout was ~0.2 s, so 1.0 s is safe. Re-check with the new model. |

### 3.4 Per-vehicle firmware values

`src/control/driving/driving.ino` — `resistance_most_left` / `resistance_most_right`
(currently 407 / 243). Re-measure per car with `check_variable_resistor.ino`.

---

## 4. Calibration procedures

### 4.1 Steering sign — do this first, it costs nothing

Motors off, node running, push the car by hand.

```bash
ros2 topic pub /yolov8_traffic_light_info std_msgs/String "data: 'Green'" --once
ros2 topic echo /topic_control_signal
```

| Car position | Expected `steering` |
|---|---|
| left of lane centre | **positive** (steers right) |
| right of lane centre | **negative** (steers left) |
| centred on a straight | near zero |

If inverted, the car leaves the lane within a metre and you will not be able to tell whether
the cause was the sign, the warp, or the gain. Check this before anything moves.

### 4.2 Bird's-eye warp — DONE 2026-08-13

Calibrated from `img/straight2.png`, `img/straight3.png`, `img/straight_lane.png`.

**Method** (repeat this if the camera is ever remounted): on a flat ground plane, lane width
in pixels is *linear* in image row — `width = 1.304·y − 119`, vanishing point at `y ≈ 91`,
consistent across all three frames. Fit the left and right boundaries of the `Lane2` mask
over the reliable band (`y = 230…380`), then read the boundary positions at a far row
(`y = 240`) and a near row (`y = 380`). Order: top-left → top-right → bottom-right → bottom-left.

The near row is capped at **380** because the bonnet occupies the bottom ~20 % of the frame.

**Result**

| | old (course default) | new |
|---|---|---|
| lane width in ROI | 273 px | **257 px** (theory 256) |
| row-to-row centre drift | 8 px | **2 px** |
| usable scan rows | **4 / 15** | **15 / 15** |

The old values placed the trapezoid's bottom edge at `y = 476`, i.e. **inside the bonnet**.
Eleven of fifteen scan rows found no lane and silently returned the `lane_width/2` fallback.

**⚠️ Caveat on `car_center_point`.** The warp maps whatever lane position the car held during
calibration onto the ROI centre. In those frames the vehicle sat about **26 px (~6 cm) left**
of true lane centre, so image `x = 320` maps to warped `x = 294`. `car_center_point` is set to
`[294, 179]` to compensate. **This assumes the camera is mounted on the vehicle centreline.**
Confirm on the car: park it deliberately centred in the lane and check that the reported lane
centre and `car_center_point` agree. If they don't, adjust `car_center_point` x — do not
re-fit `src_mat`, which is independent of lateral position.

Validate visually with the `roi_img` window: on a straight the two lane edges should be
close to **vertical and parallel**.

### 4.3 Stop parameters

Both are measurable rather than guessable, because the camera geometry is fixed.

1. Park the car where the stop sequence should *begin*. Capture a frame.
2. Run the model and read the values:

```python
from ultralytics import YOLO
r = YOLO('best.pt').predict('frame.png', conf=0.3)[0]
for b in r.boxes:
    n = r.names[int(b.cls)]
    x, y, w, h = b.xywh[0]
    if n == 'Car':      print('car_min_bbox_height =', float(h))
    if n == 'Stopline': print('stopline_trigger_y  =', float(y + h/2))
```

`creep_duration_sec` cannot be measured this way — it is how long to crawl after the line
leaves the frame, and must be tuned by trying it.

### 4.4 Steering gain

Tune at `cruise_speed 80`, then raise toward 175. The lane leaves only **8–14 cm of margin
per side**, so a gain that is merely approximately right at 80 becomes an oscillation that
leaves the lane at 175. Raise `steering_gain` until the car tracks curves without cutting
in; back off as soon as it starts weaving on straights.

---

## 5. Silent failure modes

None of these produce an error message. Each has been observed or verified.

| Symptom | Cause | Check |
|---|---|---|
| Everything runs, no detections at all | ultralytics too old for `best.pt` | look for "Error while loading model" in the `yolov8_node` log |
| `target_x` constant at `150` | lane class name mismatch — `150` is `lane_width/2`, the "no lane found" fallback | `ros2 topic echo /yolov8_lane_info` |
| Lane image blank, no path | detection-model export instead of segmentation — `mask.data` empty | `lane_extractor_node` logs a throttled "마스크가 비어 있음" warning |
| Car never starts | never sees `Greenlight` | `ros2 topic echo /yolov8_traffic_light_info` |
| Car drives forever, never stops | `Crosswalk` missed, so laps never counted | watch for `LAP n/2` in the `mission_planner_node` log |
| Car brakes repeatedly mid-lap | lane lost for > `path_timeout_sec` | look for "경로 신호 없음" warnings |
| Whole launch dies instantly | `serial_sender_node` opened `/dev/ttyACM0` at import with no Arduino attached | comment the node out for bench work |

---

## 6. Training — for the next model

```bash
yolo task=segment mode=train model=yolo26s-seg.pt \
     data={dataset.location}/data.yaml \
     epochs=150 imgsz=640 batch=32 patience=30 \
     fliplr=0.0
```

- **`fliplr=0.0` is mandatory.** `Lane1` is the left lane and `Lane2` the right; horizontal
  flip mirrors the image but keeps the labels, training the model that the left lane is
  `Lane2`. This is the most likely cause of the `Lane2` confidence dips observed at 0.37–0.69.
- Keep `imgsz=640` — all pipeline geometry lives in 640×480 pixel space.
- Roboflow export: **Instance Segmentation**, YOLOv8/v11/v26 preset (identical output). No
  auto-orient, no stretch resize, no flip augmentation.
- Box annotations for `Car` / signals / `Stopline` are fine — nothing reads their masks.
- Split by **recording session**, not randomly. Frames captured at ~10 fps are near-duplicates;
  random splits leak them across train/val and inflate the metrics.

Measured baseline on the current model, for comparison: `Lane2` present in **93 %** of video
frames, longest continuous dropout ~**0.2 s**.

---

## 7. Still unanswered

- How precisely must the car stop, and how is the run scored?
- Is the traffic light in frame from the start position, and does it stay visible while stopped?
- Must the car obey a red light on laps 1 and 2?
- Lidar orientation — untested; obstacle handling is currently dead code.
