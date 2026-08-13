# Project Context

Living context document for this autonomous vehicle project. Anything an assistant or a
new team member needs that is **not** derivable from reading the code.

Last updated: 2026-08-13

---

## 1. The mission

Run on a closed indoor track, in this sequence:

1. **Wait at the start line** until the overhead traffic light turns **green**.
2. Drive **two full laps**, staying inside `lane2` (the right-hand lane) the whole time.
3. At the end of lap 2, **stop at the stop line** near the static parked car (a white
   ride-on Mercedes G-Class). The parked car being **near** is the cue; the stop line is
   the target.

**Stop placement**: come to rest with the **stop line falling between the front and rear
wheels** — i.e. the line passes under the middle of the vehicle body.

> ⚠️ Design consequence: the camera sits on the hood looking forward, so by the time the
> stop line is under the car it has already left the bottom of the frame. The controller
> cannot see the line at the moment it must stop. This needs either (a) track the line down
> the frame, and trigger the stop a fixed travel-distance after it exits view, or (b) use
> the lidar / the parked car's apparent size for the final range. Open question.

**Hard constraints — these define failure:**
- Must not cross the **midlane** (the dashed white centre line) into the oncoming lane.
- Must not touch the **grass** (the green area outside the road).

> ❓ Scoring: pass/fail on completing the run, or is lap time / smoothness graded too?
> Is there a demo date? (revisit)

> ❓ Start position relative to the traffic light — is the light in frame from the start,
> and does it stay in frame while stopped? (revisit)

---

## 2. The track

Indoor facility, hard floor. Closed loop — a rounded rectangle with an S-bend along one
side, wrapping around a central parking area (grey, marked with parking bays, not part of
the driving route).

- **Surface**: dark grey asphalt-like mat.
- **Boundaries**: bright green "grass" mat on both the outside and inside edges of the loop.
- **Centre line**: white **dashed** stripe separating the two lanes (`midlane`).
- **Outer/inner lane edges**: solid white line in places; elsewhere the road simply meets
  the green mat.
- **Features**: a zebra crosswalk on one side, an overhead traffic light at the start area,
  red traffic cones off-road (decoration / obstacle staging).
- Curves are gentle sweeping bends plus one S-curve; no sharp corners or junctions.

### Dimensions — and why they matter

| | |
|---|---|
| Vehicle width | ~30–40 cm |
| Lane width | ~1.7 × vehicle → **~51–68 cm** |
| **Lateral margin** | **only ~8–14 cm per side** |

> ⚠️ This is tight. The current bang-bang steering in `motion_planner_node.py:106-113`
> (slope sign → ±7, nothing between) will hunt side-to-side by more than that margin on a
> straight. **Proportional steering is a requirement here, not a refinement** — a
> full-lock oscillation puts a wheel on the grass or over the midlane, which is an
> instant fail.

---

## 3. Label semantics (Roboflow)

Eight classes, exactly as named in Roboflow (**case-sensitive — see the mismatch warning
below**):

| Roboflow class | What it is | Annotation | Role |
|---|---|---|---|
| `Lane2` | **Drivable surface** of the right-hand lane | polygon | **The lane to follow** |
| `Lane1` | **Drivable surface** of the left-hand lane | polygon | Keep out — oncoming |
| `Mid` | The **painted** dashed centre line | polygon | Keep out — do not cross |
| `Greenlight` | The signal **housing, while green is lit** | box | Start trigger |
| `Red_light` | The signal **housing, while red is lit** | box | Stop / hold |
| `Stopline` | Transverse white stop line | box | **The stop target** |
| `Crosswalk` | Zebra crossing | box | Lap-counting landmark |
| `Car` | The static parked G-Class | box | Proximity cue for stopping |

~**400 images labelled**, ~800 more planned.

### ⚠️ Class-name mismatch — blocks the whole pipeline

`class_name` is taken verbatim from the model's `names` (`yolov8_node.py:168`), and every
downstream check is an **exact, case-sensitive string comparison**. Two of them do not match:

| Code expects | Roboflow has | Where | Fix |
|---|---|---|---|
| `'lane2'` | `Lane2` | `lane_info_extractor_node.py:56` | ✅ **resolved** — replaced by `lane_extractor_node`, which defaults to `Lane2` and exposes it as the `lane_class_name` parameter |
| `'traffic_light'` | *no such class* — only `Greenlight` / `Red_light` | `traffic_light_detector_node.py:62`, `motion_planner_node.py:91` | **change the code** (see below) |

As-is, the lane pipeline sees zero `lane2` detections → blank edge image → no path → no
steering. It fails **silently**, with no error anywhere — measured: it publishes
`target_x = [150, 150, 150]` (`lane_width/2`, the "no lane found" fallback) indefinitely.

**Both are now fixed in code, so the Roboflow class names need no changes.** Keep labelling
the remaining 800 images with the names exactly as they are.

### ✅ Decided: `traffic_Light` is dropped

The signal head is labelled **only** as `Greenlight` or `Red_light` — the state lives in the
class name and the housing is never labelled state-agnostically. This removes the
contradictory supervision that would have come from labelling the same housing three
different ways, and leaves the two state classes clean across all ~1200 images.

Consequence: **no class named `traffic_light` exists at all**, so the two code sites that
compare against that string are dead and must be rewritten (below).

**Boxes are fine — in fact they are the right choice for the last three.** Nothing in the
pipeline reads a mask for those classes: `get_traffic_light_color()` samples HSV inside
`detection.bbox`, and `motion_planner_node.py:92-95` reads bbox corners. A Roboflow
instance-segmentation project exports box annotations as 4-point rectangular polygons, which
satisfies the seg model's label format at zero cost. Only `lane1`/`lane2`/`midlane` need
true polygons.

> ⚠️ Verify at export time that all six classes are in **one Instance Segmentation project**.
> Segmentation and detection projects cannot be mixed, and a box-only export kills the lane
> pipeline (empty `mask.data` → blank edge image).

**Important consequence:** because `lane2` is the *drivable surface*, its mask outline
already yields **both** boundaries of the correct lane — the midlane on the left and the
grass edge on the right. So `draw_edges(..., cls_name='lane2')` as written in
`lane_info_extractor_node.py:56` is **already correct and should be left alone**.
Do **not** additionally render `lane1` — merging both lane surfaces into one blob would
centre the car on the midlane, i.e. straddling the centre line.

### The HSV traffic-light detector must be replaced

Because the lamp state is baked into the class name, the light state is readable straight off
`/detections`. `traffic_light_detector_node` currently crops the signal bbox and classifies
it by HSV pixel ratio (`camera_perception_func_lib.py:183-216`) — that whole step goes away,
and with it the node's `image_raw` subscription, its `CvBridge`, and the
`ApproximateTimeSynchronizer`. The node reduces to a class-name lookup over `DetectionArray`.

The network is the more robust detector here anyway: the venue photo shows the whole scene
taking a strong colour cast from the illuminated signal, which is exactly what skews a raw
HSV ratio.

Planned mapping (keeps the published `String` contract that `motion_planner` expects):

| Detected class | Published on `yolov8_traffic_light_info` |
|---|---|
| `Red_light` | `"Red"` |
| `Greenlight` | `"Green"` |
| neither | `"None"` |

`motion_planner_node.py:91` must likewise match `Red_light` instead of `traffic_light` when
it reaches for the bbox to judge proximity.

**No amber/yellow** exists on this course and there are no images of one, so the `'Yellow'`
branch in the HSV code is dead and needs no replacement.

---

## 4. Hardware

- **Vehicle**: white ride-on style car, Arduino-driven, potentiometer feedback on steering.
- **Camera**: mounted low on the hood, forward-facing with a slight downward tilt; the car's
  own hood is visible in the bottom ~15% of the frame. 640×480. Model unknown.
- **Lidar**: **physically mounted** — a puck sensor sits on the hood centre, cabled.
  **Not yet tested in software.** Orientation (which direction is 0°) unknown.
- **Compute**: laptop with **NVIDIA GTX 1660** → CUDA is available. Set `yolov8_node`'s
  `device` parameter to `cuda:0` (currently `"cpu"`, `yolov8_node.py:60-62`).

### ⚠️ ultralytics version requirement

`best.pt` is a **YOLO26n-seg** model (2.69 M params, 9.1 GFLOPs) trained with **ultralytics
8.4.118**. It uses `C3k2` / `C2PSA` blocks, which do not exist in older ultralytics builds.

The workspace currently has **ultralytics 8.2.69**, which cannot deserialise the file:

```
AttributeError: Can't get attribute 'C3k2' on <module 'ultralytics.nn.modules.block'>
```

`yolov8_node.on_activate()` catches this, logs "Error while loading model", returns
`TransitionCallbackReturn.FAILURE`, and the node then publishes nothing at all — every
downstream node sits idle with no obvious cause.

**Fix**: pin the training version exactly on the vehicle laptop.

```bash
pip install ultralytics==8.4.118
```

Do not merely bump to ≥ 8.3.0 — the YOLO26 architecture is newer than that. Matching the
version that produced the weights removes the guesswork entirely.

**No code change is required.** Every ultralytics API that `yolov8_node.py` and
`yolov8_visualizer_node.py` call was verified against 8.4.118 with these weights:
`YOLO()`, `.fuse()`, `.predict(source, verbose, stream, conf, device)`, `results[0].cpu()`,
`len(results)`, `boxes.cls/.conf/.xywh`, `masks[i].xy[0]`, `results.orig_img.shape`,
`yolo.names`, and the `Results`/`Boxes`/`Masks`/`Keypoints`/`Annotator`/`colors` imports.
All work unchanged, despite the node's `yolov8` name.

**Why the model came out as YOLO26**: the training command omitted `model=`, so ultralytics
used its default segmentation checkpoint for the installed version. This is fine — do not
retrain to force YOLOv8.

`install.sh` installs ultralytics unpinned, so a fresh setup gets whatever is current; this
laptop simply has an older build from an earlier install.
- **Traffic light**: a real, full-size overhead signal head mounted high above the start
  area — large in frame, unambiguous colour.

---

## 5. Vehicle behaviour

- Steering: **−7 … +7** discrete steps, closed-loop against the potentiometer. Confirmed
  working. **No noticeable drift-back** — the wheel holds its commanded angle.
- Speed: independent left/right, **reverse supported** (negative values).
- Near-maximum speed was still controllable but hard to manage. **~70% of full is the
  balanced target** → roughly **175** on the 0–255 scale (`motion_planner_node.py` currently
  hardcodes 100).

> ❓ Lock-to-lock steering sweep time? Needed to know whether the 0.1 s motion-planner tick
> can actually keep up with commanded angles.

> ❓ Any left/right asymmetry or dead zone in steering?

---

## 6. Obstacles

Not confirmed. Static obstacles may appear. Red cones are present at the venue but currently
staged off-road. The parked car is static and is a *goal*, not an obstacle to avoid.

---

## 7. Gaps between the mission and the current code

The shipped pipeline is a pure lane-follower. Three mission requirements have **no
implementation at all** yet:

1. **Start gating on green.** `motion_planner_node` only reacts to `'Red'`; it has no
   "wait until green, then latch into driving" state. Right now it would drive off
   immediately, and `traffic_light_detector_node` is commented out of the launch file.
2. **Lap counting.** Nothing counts laps. There are no wheel encoders, so dead reckoning is
   unreliable. The **crosswalk is labelled**, appears exactly once per lap, and is visually
   unmistakable — that is the landmark to use. Needs debouncing (a minimum time or travel
   between counts) so a single pass, seen over many frames, increments only once.
3. **Stopping at the stop line.** `Stopline` and `Car` are both labelled, so detection is
   solved; what remains is placing the line under the vehicle's midpoint despite the
   camera blind spot — see §1.

This implies `motion_planner_node.timer_callback` should become an explicit **state machine**
(`WAIT_GREEN → LAP_1 → LAP_2 → APPROACH_CAR → STOPPED`) rather than the current flat
if/elif chain of instantaneous reactions.

Also still open from the code review:
- `best.pt` not yet trained or deployed to the workspace root.
- `serial_sender_node`, the three lidar nodes, and `traffic_light_detector_node` are all
  commented out in `launch_pkg/launch/main.launch.py`.
- `motion_planner_node` dereferences `self.detection_data` while it can still be `None`, and
  its red-light branch can leave stale speed commands.

---

## 8. Established from the code (verified)

### Pipeline
```
image_raw ─► yolov8_node ─► detections ─┬─► lane_info_extractor ─► yolov8_lane_info
                                        │        (bird's-eye warp, ROI crop, lane center)
                                        │                    ▼
                                        │            path_planner ─► path_planning_result
                                        ├─► traffic_light_detector ─► yolov8_traffic_light_info
lidar_raw ─► processor ─► obstacle_detector ─► lidar_obstacle_info
                                        └────────────► motion_planner ─► topic_control_signal
                                                                              ▼
                                                            serial_sender ─► Arduino
```

### Hard contracts
- **Model**: `best.pt` at the workspace root (path resolves against CWD). Must be a
  YOLOv8 **segmentation** model — `draw_edge()` consumes `detection.mask.data` polygons.
- **Class name strings** are the only thing matched at runtime (class IDs are ignored):
  - `lane2` → `lane_info_extractor_node.py:56`
  - `traffic_light` → `traffic_light_detector_node.py:62`, `motion_planner_node.py:91`
  - Any other class is published on `/detections` and silently ignored.
- **Image size 640×480** is baked into every calibration constant.
- **Serial protocol**: `s{steering}l{left_speed}r{right_speed}\n` @ 115200 on `/dev/ttyACM0`.
- The `lib/*.py` sources are **mirrors**; `lib/__init__.py` loads the `.pyc`. Editing the
  `.py` has no effect. Requires Python 3.10.

### Lane-center algorithm (`get_lane_center`)
Scans a 10-px horizontal band of the bird's-eye ROI, sorts non-zero x-coords, finds the
**single widest gap**, returns its midpoint. If the gap is narrower than `lane_width/3` it
assumes one visible line and guesses the centre by offsetting `±lane_width/2` using the sign
of the road gradient.

### Data collection tool
`src/data_collection/data_collection.py` (logic in a `.pyc`). Keys: `w`/`s` speed ±10
(cap ±250), `a`/`d` steer ∓1 (cap ±7), `r` reset, `c` capture frame, `f` quit. Writes
`Collected_Datasets/<timestamp>/{n}_steer:{s}_left_speed:{l}_right_speed:{l}.png` at 640×480.
The steering/speed labels are **not used** by this pipeline. Known bug: the `right_speed`
field in the filename actually contains `left_speed`.
