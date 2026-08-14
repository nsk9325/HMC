# COMMANDS

Every command needed to build, run, tune and diagnose the vehicle.

Substitute the workspace path for your machine — `~/ros2_ws` here, `~/HMC` on the migrated
laptop. **Always run from the workspace root**: `best.pt` and the bundled video path resolve
against the working directory.

Related: [notice.md](notice.md) · [migration.md](migration.md) · [project.md](project.md) · [changes.md](changes.md)

---

## 1. Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Expect **7 packages finished**. `UserWarning: Unknown distribution option: 'tests_require'`
is harmless.

Python-only edits need no rebuild (`--symlink-install`). Only `.msg`, `CMakeLists.txt` or
`setup.py` changes do.

Every new terminal needs:

```bash
cd ~/ros2_ws && source install/setup.bash
```

---

## 2. Run modes

All six use the same launch file; the flags differ.

| # | Mode | Light gate | Laps | Log |
|---|---|---|---|---|
| 1 | Tuning | off | 2 | off |
| 2 | Tuning + log | off | 2 | on |
| 3 | Mission + log | on | 2 | on |
| 4 | Contest | on | 2 | off |
| 5 | Braking test | off | **0** | off |
| 6 | Braking test + log | off | **0** | on |

### 1. Tuning — no light gate, no log
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false wait_for_green:=false
```

### 2. Tuning — no light gate, with log
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false wait_for_green:=false debug_log:=true
```

### 3. Mission run — light gate, with log
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false debug_log:=true
```

### 4. Contest run — light gate, no log
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false
```

### 5. Braking test — no log
Place the car a few metres short of the goal. `target_lap_count:=0` enters `APPROACH`
immediately.
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false \
    wait_for_green:=false target_lap_count:=0
```

### 6. Braking test — with log
```bash
ros2 launch launch_pkg mission.launch.py \
    use_serial:=true data_source:=camera show_image:=false \
    wait_for_green:=false target_lap_count:=0 debug_log:=true
```

### Bench — car physically cannot move
`use_serial` defaults to `false`, so no command ever reaches the Arduino.
```bash
ros2 launch launch_pkg mission.launch.py data_source:=camera
```
On the bundled video instead of the camera (perception smoke test only — the calibration is
for the vehicle camera, so steering quality means nothing here):
```bash
ros2 launch launch_pkg mission.launch.py
```

### Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `use_serial` | `false` | send commands to the Arduino (**the car moves**) |
| `data_source` | `video` | `camera` for the real vehicle |
| `wait_for_green` | `true` | `false` skips the start gate and drives immediately |
| `target_lap_count` | `2` | `0` enters `APPROACH` at once, for braking tests |
| `debug_log` | `false` | write `debug_logs/mission_<t>.csv` |
| `show_image` | `true` | debug windows — turn **off** on the vehicle, they cost frame rate |
| `device` | `cuda:0` | `cpu` if CUDA isn't available (~3 Hz, too slow to drive) |
| `threshold` | `0.3` | YOLO detection confidence |

⚠️ Modes 3 and 4 count laps off the **traffic light**, and the latch starts engaged — so the
car must **start under the light**. Starting elsewhere makes the first sighting count as lap 1
and it stops a lap early.

---

## 3. Pause key

Second terminal. `p` toggles, `q` quits the listener.

```bash
cd ~/ros2_ws && source install/setup.bash
echo "p = 일시정지 토글 / q = 종료"
while true; do
  read -rsn1 k
  case "$k" in
    p|P) ros2 service call /mission_planner_node/toggle_pause std_srvs/srv/Trigger {} \
           2>/dev/null | grep -o "message='[^']*'" ;;
    q|Q) echo "listener 종료"; break ;;
  esac
done
```

Single toggle without the listener:
```bash
ros2 service call /mission_planner_node/toggle_pause std_srvs/srv/Trigger {}
```

Notes:
- **Do not use spacebar.** bash `read` strips whitespace with the default `IFS`, so a space
  arrives as an empty string and never matches. Letter keys are unaffected.
- Pause zeroes the drive command but keeps the state machine — lap count, ARM latch and
  `APPROACH` progress all survive, and the timers are shifted by the paused duration on resume.
- **~1 s latency** (`ros2 service call` starts a node per invocation). This is for
  repositioning between attempts, **not an emergency stop**. Use Ctrl-C or the power switch.
- Pause does not brake; the car coasts to a halt.

---

## 4. Monitoring

```bash
cd ~/ros2_ws && source install/setup.bash

ros2 topic hz /image_raw                    # ~20 Hz (TIMER 0.05)
ros2 topic hz /detections                   # >= 10 Hz, else perception is the bottleneck
ros2 topic hz /topic_control_signal         # steady 10 Hz regardless of load
ros2 topic echo /topic_control_signal       # steering within ±2 on straights
ros2 topic echo /yolov8_lane_info           # target_x must VARY, not sit at 128
ros2 topic echo /yolov8_traffic_light_info  # Red / Green / None
ros2 node list | grep serial                # serial_sender_node must be present
```

Fake a green light (for modes 3/4 away from the signal):
```bash
ros2 topic pub -r 10 /yolov8_traffic_light_info std_msgs/String "data: 'Green'"
```
Use `-r 10` for a couple of seconds, not `--once` — a one-shot publisher often exits before
discovery completes and the message is never delivered. The planner latches on the first
`Green`, so a moment is enough.

---

## 5. Live tuning

Parameters take effect **immediately** — the node has an `on_set_parameters` callback and logs
each change. (Before that callback existed, `ros2 param set` silently did nothing.)

```bash
cd ~/ros2_ws && source install/setup.bash

# 주행
ros2 param set /mission_planner_node cruise_speed 180
ros2 param set /mission_planner_node steering_gain 0.25       # 곡선에서 부족하면 올림
ros2 param set /mission_planner_node far_weight 1.0           # 언더스티어 지속 시
ros2 param set /mission_planner_node steering_smoothing 0.5   # 직선에서 흔들리면 낮춤
ros2 param set /mission_planner_node turn_slowdown 0.45       # 곡선 감속 강화

# 정지
ros2 param set /mission_planner_node car_stop_bbox_height 215 # 짧게 서면 올림
ros2 param set /mission_planner_node approach_speed 100       # 접근이 너무 빠르면 낮춤
ros2 param set /mission_planner_node min_move_speed 30        # 목표 전에 멈춰버리면 올림
ros2 param set /mission_planner_node max_reverse_speed 80     # 후진 제동 활성화 (기본 0)
ros2 param set /mission_planner_node stopline_latch_sec 12.0

# 랩 카운트
ros2 param set /mission_planner_node lap_landmark_min_height 30
ros2 param set /mission_planner_node lap_landmark_clear_sec 5.0

# 차선 상실 시 동작
ros2 param set /mission_planner_node blind_forward_sec 3.0
ros2 param set /mission_planner_node blind_forward_speed 70
```

Check current values:
```bash
ros2 param list /mission_planner_node
ros2 param get /mission_planner_node car_stop_bbox_height
ros2 param dump /mission_planner_node
```

**Startup-only** — these are read once and need a relaunch: `wait_for_green`, `debug_log`,
`timer`, all topic names, and everything in `lane_extractor_node` (`src_mat`, `lane_width`,
`cutting_idx`, `lane_class_name`).

---

## 6. Logs

```bash
ls -t debug_logs/*.csv | head -1                    # 최신 실행
```

Columns: `t, state, lap, traffic_light, steering, left_speed, right_speed, path_age, n_det,
detections`. The `detections` field is `class:conf:height:bottom_y` per class.

```bash
# 상태 전이만 추출
awk -F, 'NR==1||$2!=p{print $1,$2,$3,$4; p=$2}' debug_logs/mission_*.csv | tail -20

# 왜 출발하지 않았는가
cut -d, -f2,4,10 debug_logs/mission_*.csv | grep WAIT_GREEN | tail -20

# 조향이 ±7 에 붙어 있던 비율
awk -F, 'NR>1 && ($5>=7||$5<=-7){n++} END{print n" ticks = "n/10"s"}' debug_logs/mission_*.csv

# 정지 구간에서의 Car / Stopline 기하
grep -oE "(Car|Stopline):[0-9.]+:[0-9]+:[0-9]+" debug_logs/mission_*.csv | tail -20
```

---

## 7. Diagnostics

```bash
# 모델
python3 -c "from ultralytics import YOLO; m=YOLO('best.pt'); print(m.task, m.names)"

# 환경
python3 -c "import numpy, setuptools, packaging, torch; \
print(numpy.__version__, setuptools.__version__, packaging.__version__, torch.cuda.is_available())"
python3 -c "import cv_bridge, cv2; print('cv_bridge OK', cv2.__version__)"

# 장치와 권한
ls /dev/video* /dev/ttyACM* /dev/ttyUSB*
id -nG | grep dialout
python3 -c "import os; print(os.access('/dev/ttyACM0', os.W_OK))"
```

Expected model output:
```
segment {0:'Car', 1:'Crosswalk', 2:'Greenlight', 3:'Lane1', 4:'Lane2',
         5:'Mid', 6:'Orange_light', 7:'Red_light', 8:'Stopline'}
```

### Silent failures — none of these print an error

| Symptom | Cause | Check |
|---|---|---|
| Everything runs, no detections | ultralytics too old for `best.pt` | `yolov8_node` log: "Error while loading model" |
| `target_x` constant at 128 | lane not found — 128 is `lane_width/2`, the fabricated fallback | `ros2 topic echo /yolov8_lane_info` |
| Runs, car never moves | not in `dialout`; `serial_sender_node` died at startup | `ros2 node list \| grep serial` |
| Never starts | never sees `Greenlight` | `ros2 topic echo /yolov8_traffic_light_info` |
| Drives forever | laps not counting — light not seen or latch stuck | `LAP n/2` lines in the terminal |
| Stops a lap early | didn't start under the traffic light | first `LAP 1/2` timestamp |
| Whole launch dies instantly | Arduino absent; the port is opened at import | comment out `serial_sender_node` |

---

## 8. Data collection

```bash
python3 src/data_collection/data_collection.py
```
`w`/`s` speed ±10 · `a`/`d` steer ∓1 · `r` reset · `c` capture frame · `f` quit.
Frames land in `src/camera_perception_pkg/camera_perception_pkg/lib/Collected_Datasets/<timestamp>/`.

Also the easiest way to measure `min_move_speed`: wheels **on the ground**, press `w` one step
at a time and note the first value that moves the car.

---

## 9. Training

```bash
yolo task=segment mode=train model=yolo26s-seg.pt \
     data={dataset.location}/data.yaml \
     epochs=150 imgsz=640 batch=32 patience=30 \
     fliplr=0.0
```

`fliplr=0.0` is **mandatory** — `Lane1` is the left lane and `Lane2` the right, so a horizontal
flip mirrors the image but keeps the labels, teaching the model that the left lane is `Lane2`.
Keep `imgsz=640`; all pipeline geometry lives in 640×480 pixel space.

Then copy `runs/segment/train/weights/best.pt` to the workspace root and verify with §7.
