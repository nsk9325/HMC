# Changes

Every modification made to this workspace, newest first.

**Ground rule for this log**: the original course files are never edited. Replacements are
added as new files alongside the originals, and the original stays in the tree, unrun.

---

## 2026-08-13 — Mission state machine + traffic-light state detection

### Files added

| File | Replaces | Status |
|---|---|---|
| `src/camera_perception_pkg/camera_perception_pkg/lane_extractor_node.py` | `lane_info_extractor_node.py` | new |
| `src/camera_perception_pkg/camera_perception_pkg/traffic_light_state_node.py` | `traffic_light_detector_node.py` | new |
| `src/decision_making_pkg/decision_making_pkg/mission_planner_node.py` | `motion_planner_node.py` | new |
| `src/launch_pkg/launch/mission.launch.py` | `main.launch.py` | new |
| `project.md` | — | new (project context doc) |
| `changes.md` | — | new (this file) |

### Files modified

Only entry-point registration — one added line each, no existing line changed:

| File | Change |
|---|---|
| `src/camera_perception_pkg/setup.py` | `+ 'lane_extractor_node = camera_perception_pkg.lane_extractor_node:main',` |
| `src/camera_perception_pkg/setup.py` | `+ 'traffic_light_state_node = camera_perception_pkg.traffic_light_state_node:main',` |
| `src/decision_making_pkg/setup.py` | `+ 'mission_planner_node = decision_making_pkg.mission_planner_node:main',` |

The original entries are untouched, so `traffic_light_detector_node`, `motion_planner_node`
and `main.launch.py` all still work exactly as before. The packages now expose 5 and 3
executables respectively, instead of 4 and 2.

`src/launch_pkg/CMakeLists.txt` already does `install(DIRECTORY launch ...)`, so
`mission.launch.py` needed no CMake change.

### Build and verification performed

```
colcon build --symlink-install --packages-select camera_perception_pkg decision_making_pkg launch_pkg
```

- Both new executables appear in `install/*/lib/*/` and start under `ros2 run`.
- `mission.launch.py` installed to `install/launch_pkg/share/launch_pkg/launch/`.
- Node construction, the no-data tick, the green-light transition, steering sign, the slew
  limiter and corner slowdown were exercised directly — see "Verified behaviour" below.
- Ctrl-C exits cleanly with no traceback.

---

### 1. `lane_extractor_node.py`

**Why**: the dataset class is `Lane2` (capital L); the original hardcodes `'lane2'`.
`class_name` comparison is case-sensitive, so the original finds nothing. Rather than
renaming 400 existing annotations in Roboflow, the class name became a parameter.

**What changed**
- `lane_class_name` parameter, default `'Lane2'`.
- Calibration constants promoted to ROS parameters so they can be tuned trackside without a
  rebuild: `src_mat` (flattened 8 ints), `cutting_idx`, `lane_width`, `theta_limit`,
  `detection_thickness`.
- Replaced `CPFL.draw_edges()` with an in-node loop over `CPFL.draw_edge()`. The library
  function takes its canvas size from `detections[0]`, which in a multi-class model may be a
  `Greenlight` box rather than a lane. The node now scans for the first detection carrying a
  non-empty mask, and warns (throttled) if none does — which is also the early warning that a
  detection-only model was exported by mistake.

**Unchanged on purpose**: publishes `LaneInfo` on `yolov8_lane_info` and the ROI image on
`roi_image`, identical to the original.

**Demonstration of the original failure mode** — same synthetic input, two class names:

| `lane_class_name` | published `target_x` |
|---|---|
| `Lane2` | `[378, 376, 374]` — tracks the lane |
| `lane2` | `[150, 150, 150]` |

`150` is `lane_width / 2`, the "no lane found" fallback inside `get_lane_center`. It
publishes at that constant indefinitely with no warning, which is why the mismatch is worth
this much care.

---

### 2. `traffic_light_state_node.py`

**Why**: the dataset labels the signal *housing* by lit state — `Greenlight` / `Red_light` —
and the class `traffic_light` was dropped entirely. The original node matched the string
`'traffic_light'` and then classified the colour by counting HSV pixels inside the bbox. That
string now never matches, so the original node is permanently dead.

**What changed**
- Light state is read from the class name; the HSV analysis is gone.
- Removed the `image_raw` subscription, `CvBridge`, and `ApproximateTimeSynchronizer` —
  the node no longer needs images at all, only `DetectionArray`.
- When several signals are detected in a frame, the highest-scoring one wins, so a momentary
  simultaneous red+green detection still yields a single state.
- Added a `min_score` parameter (default 0.5) to reject weak detections.

**Unchanged on purpose**: publishes `String` on `yolov8_traffic_light_info` with values
`"Red"` / `"Green"` / `"None"` — identical to the original contract, so any subscriber works
without modification.

**Net effect**: ~40 fewer lines and one less image subscriber in the graph. Also more robust
than HSV here: the lit signal casts a strong colour over the whole scene, which is exactly
what skews a raw HSV ratio.

---

### 3. `mission_planner_node.py`

**Why**: the original `motion_planner_node` was a flat if/elif of instantaneous reactions with
no memory of mission progress. The mission needs sequence: wait for green → two laps → stop.

**State machine**

```
WAIT_GREEN ──(green)──> DRIVING ──(2 laps)──> APPROACH
                                                 │
                               (stop line reaches bumper)
                                                 ▼
                                               CREEP ──(timer)──> STOPPED
```

**What was added**

1. **Start gate** — holds at zero until `Green` is seen, then latches into `DRIVING`.
   The original had no gate and would drive off immediately.
2. **Lap counting** — counts `Crosswalk` detections whose bbox bottom crosses
   `crosswalk_trigger_y`. Latched plus a `lap_cooldown_sec` guard, so one physical pass seen
   over many frames increments exactly once.
3. **Stop sequence** — `APPROACH` slows down and waits for the parked `Car` to be near
   (bbox height ≥ `car_min_bbox_height`) *and* `Stopline` to reach the bottom of the frame.
   Then `CREEP` drives straight briefly to place the line under the car's midpoint, because
   the line is in the camera's blind spot at the moment it matters. `STOPPED` holds zeros.
4. **Proportional steering** — replaces the bang-bang `±7` on the sign of the slope. Takes a
   lookahead point on the planned path, computes its bearing with `atan2`, scales by
   `steering_gain`, clamps to `±7`, and rate-limits the change per tick (`steering_slew`).
   Required, not cosmetic: the lane leaves only ~8–14 cm per side, and a full-lock oscillation
   puts a wheel on the grass or over the midlane.
5. **Corner slowdown** — speed scales down with steering magnitude via `turn_slowdown`.
6. **Path staleness stop** — if no new path for `path_timeout_sec` (lane detection lost),
   the car stops instead of coasting blind.
7. **Cruise speed 100 → 175**, matching the measured "~70 % of full is controllable".

**Bugs fixed from the original**
- `self.detection_data.detections` was dereferenced while it could still be `None`
  (`motion_planner_node.py:90`) — an `AttributeError` on the first red light seen before any
  detection arrived. All detection access now goes through a `None`-safe accessor.
- The red-light branch never assigned the speed commands, so if `Red` was published but no
  matching bbox was found, the previous speeds persisted and the car kept driving. Command
  generation is now a single function that always returns a complete triple.
- Steering used `DMFL.calculate_slope_between_points`, which returns the **string** `'inf'`
  when both y values match; the subsequent `> 0` comparison would raise `TypeError`.
  Replaced with `atan2`, which handles the vertical case natively.

**Behaviour deliberately left configurable**
- `stop_on_red_while_driving` defaults to **False**. On this course the signal is a start
  gate, so honouring red mid-lap could stop the car needlessly on laps 1 and 2. Set True if
  the rules require obeying the light every pass. ← *needs confirming with the graders*
- Lidar obstacle handling is retained as an emergency stop that overrides every state, but
  the lidar nodes are not launched yet, so `lidar_data` stays `None` and it never fires.

**Verified behaviour** (exercised directly, not just compiled)

| Check | Result |
|---|---|
| Tick with no data at all | returns `0,0,0` — the original raised `AttributeError` here |
| `Green` published | latches `WAIT_GREEN → DRIVING` |
| Path bending left / right | steering `−7` / `+7` — sign convention matches `driving.ino` |
| Slew limiter | ramps 2 steps per tick instead of snapping to full lock |
| Corner slowdown | 175 at centre → 96 at full lock |
| Path goes stale | warns and commands zero speed |
| Ctrl-C | clean exit, no traceback |

**Shutdown fix** — both new nodes guard `rclpy.shutdown()` with `if rclpy.ok()`. The original
course pattern calls it unconditionally in a `finally`, but on Ctrl-C rclpy's signal handler
has already shut the context down, so the call raises
`RCLError: rcl_shutdown already called` and the process exits non-zero. Cosmetic, but it
makes every Ctrl-C print a traceback and reports node failure to `ros2 launch`.

**Numbers that must be calibrated on the real track** (all are ROS parameters):
`crosswalk_trigger_y` (300), `car_min_bbox_height` (120), `stopline_trigger_y` (430),
`creep_duration_sec` (0.8), `steering_gain` (0.35), `lookahead_index` (40).
The three stop-related ones determine where the car actually comes to rest and are pure
guesses until measured.

---

### 4. `mission.launch.py`

Same graph as `main.launch.py` but wired to the new nodes, with `traffic_light_state_node`
enabled (needed for the start gate) and `serial_sender_node` enabled (needed to actually
drive). Lidar nodes stay commented out until the hardware is verified in software.

> ⚠️ `serial_sender_node` opens `/dev/ttyACM0` at **import** time, so this launch file fails
> immediately if the Arduino is not connected. Comment that node out for bench testing.

---

## Still outstanding

- ~~`Lane2` → `lane2` rename in Roboflow~~ — resolved in code instead; `lane_extractor_node`
  defaults to `Lane2`. Roboflow class names now need no changes at all.
- Label remaining ~800 images, export, train, place `best.pt` at the workspace root.
- Calibrate `src_mat` / `cutting_idx` / `lane_width` / `CAR_CENTER_POINT` on the real camera.
- Verify lidar orientation and enable the lidar nodes.
- Confirm stop tolerance, scoring, and the start position relative to the signal.
