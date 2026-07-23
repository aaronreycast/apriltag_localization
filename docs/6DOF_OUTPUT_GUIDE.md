# Open Item #3 — 6-DoF Pose Output: Implementation Guide

_Prepared 2026-07-15 from an audit of `localizer_node.py`. Pick this up in a fresh session to implement Item #3._

Goal: extend `vessel_localizer` from `geometry_msgs/Pose2D` (x, y, yaw) to a full
**6-DoF** vessel pose (x, y, z + roll, pitch, yaw) on `/vessel_pose`.

---

## 1. Audit findings (the starting point)

**The 6-DoF math already exists — it's just being thrown away.**
`on_tags` computes the full SE(3) chain at [localizer_node.py:194](../apriltag_localization/localizer_node.py#L194):
```python
T_W_B = T_W_T @ invert_T(T_C_T) @ invert_T(self.T_B_C)   # full 4x4
```
then flattens it to `(x, y, yaw)` at lines 196–198, discarding **z, roll, pitch**.
So the core transform work is done; this item is about *carrying* the full pose
through the message, the fusion, and the timestamp — not re-deriving geometry.

**Blast radius is essentially nil inside this repo.** The only workspace
references to `/vessel_pose` / `Pose2D` are the node itself plus docs
(`README.md`, `docs/PHASE3_LOCALIZATION_RESULTS.md`, `package.xml` description).
**No in-workspace subscriber exists.** `grep -rn vessel_pose src/` confirms.

**The real consumer is external** — the primary lidar+IMU nav/EKF that this
system feeds to correct drift (see PROJECT_STATE motivation). It is NOT in this
workspace, so **the message-type choice must be coordinated with whoever owns
that EKF** (see Step 0). That is the one true blocker.

**Current gaps that 6-DoF must fix along the way:**
- **No timestamp / frame_id.** `Pose2D` has no header; the node publishes with no
  `stamp` and no `frame_id`. An EKF fusing this needs the **image acquisition
  time** (copy `msg.header.stamp` from the `/tags` message) and a named world
  frame. This is mandatory for time-synchronized fusion.
- **Fusion only handles x, y, yaw** (`fuse`, lines 227–243). Needs z (trivial
  weighted average) and full rotation averaging (the real work).
- **Camera mount `T_B_C` is a placeholder** (`[0.20, 0.0, 0.5]`, Item #6). In 2D
  only its x/y mattered; in 6-DoF its **z and any mount roll/pitch feed straight
  into reported z/roll/pitch.** 6-DoF is only as good as this mount. Couple the
  two items mentally.

---

## 2. Design decisions

- **Message type: `geometry_msgs/PoseWithCovarianceStamped`.** It carries the
  full pose + `header` (stamp, frame_id) + a 6×6 covariance an EKF wants. It's in
  `geometry_msgs` — **no custom `.msg` package needed** (the old PROJECT_STATE
  note about a "new message type" is satisfied by a standard type). If the
  external EKF cannot take covariance, fall back to `PoseStamped`; the code below
  is identical minus the covariance block.
- **`frame_id`**: the world/tank frame. Pick a stable name (`"map"` is the ROS
  convention for a drift-free world fix; `"tank"` if you prefer explicit). Use it
  consistently and document it.
- **`stamp`**: copy from the incoming `/tags` message header
  (`msg.header.stamp`) — the pose is valid at image-capture time, not publish
  time. **Do not use `now()`.**
- **Rotation representation on the wire**: quaternion (as PoseWithCovariance
  requires). Keep the internal chain in rotation matrices; convert once at the
  end.
- **No new package dependencies required** for the core: `geometry_msgs` is
  already pulled in. For matrix→quaternion, either add a tiny local helper (no
  dep) or use `scipy.spatial.transform.Rotation` if scipy is already on the
  Jetson (check `python3 -c "import scipy"`). Prefer the local helper to avoid a
  new runtime dep.

---

## 3. Step-by-step

### Step 0 — Confirm the external EKF contract (BLOCKER)
Before writing code, confirm with the owner of the primary nav/EKF:
- Expected message type (`PoseWithCovarianceStamped` vs `PoseStamped` vs a custom
  interface), topic name, and expected `frame_id`.
- Whether they want covariance, and in what frame the orientation is expressed.
Everything below assumes `PoseWithCovarianceStamped` on `/vessel_pose` in `"map"`.

### Step 1 — Swap the message type + add header/stamp
- Import: `from geometry_msgs.msg import PoseWithCovarianceStamped`
  (drop the `Pose2D` import at line 12).
- Publisher (line 89): `self.pub = self.create_publisher(PoseWithCovarianceStamped, '/vessel_pose', 10)`.
- Declare a `frame_id` parameter (default `"map"`) so it's configurable.
- In `on_tags`, keep the incoming `msg.header.stamp` around to stamp the output.

### Step 2 — Extract full SE(3) per tag (stop flattening)
In the per-tag loop (lines 196–201), instead of storing `(x, y, yaw)`, store the
**full `T_W_B`** (or its translation vector + rotation matrix) so nothing is lost:
```python
estimates.append(T_W_B)         # keep the whole 4x4
weights.append(det.decision_margin)
```
The 2D debug logging (lines 209–222) can still derive x/y/yaw from `T_W_B` for the
console.

### Step 3 — Extend fusion to z + rotation
Rewrite `fuse` to take the list of `T_W_B`:
- **Translation**: weighted average of the three `T[:3,3]` vectors (x, y, z) —
  same idea as today, just include z.
- **Rotation (the real work)**: weighted average of the rotation matrices. Use
  **Markley's quaternion averaging**: convert each `R` to a quaternion `q_i`,
  build `M = Σ w_i q_i q_iᵀ` (4×4), and take the eigenvector of the largest
  eigenvalue as the mean quaternion. Handle the `q` / `-q` double-cover (flip
  each `q_i` to the same hemisphere before accumulating). For the common 1–2 tag
  case this is cheap and robust. (Simpler stopgap: if only one tag, pass its
  rotation through untouched.)
- Return a fused `(t_xyz, quaternion)`.

### Step 4 — Populate and publish
```python
out = PoseWithCovarianceStamped()
out.header.stamp = msg.header.stamp        # image-capture time
out.header.frame_id = self.frame_id        # "map"
out.pose.pose.position.x/y/z = t_xyz
out.pose.pose.orientation = quaternion     # x,y,z,w
out.pose.covariance = cov                  # 6x6 row-major, see Step 5
self.pub.publish(out)
```

### Step 5 — Covariance (informed by Item #2 — nice tie-in)
Item #2 gave the real error budget: **sub-2 cm through ~60–70% of FOV, ~5–9%
depth over-read at the extreme edge.** Turn that into a per-estimate covariance
rather than a flat guess:
- Base position variance from the sub-2 cm floor; **inflate with off-axis
  distance** (use the tag's image eccentricity — reuse `u_off` from
  `edge_fov_probe`, i.e. tag-center column minus `cx`) so edge tags are trusted
  less, matching the measured depth bias.
- Scale inversely with `decision_margin` (weaker detections → larger variance).
- This can start as a documented heuristic and be refined later. If the external
  EKF ignores covariance, skip and use `PoseStamped`.

### Step 6 — Respect the mount coupling (Item #6)
Flag in code + docs that reported **z, roll, pitch are only as trustworthy as
`T_B_C`** (`make_camera_mount`, lines 116–133), whose translation is still a
placeholder and whose rotation assumes a perfectly forward, level camera. Any
real mount pitch/roll will show up as a constant offset in the 6-DoF output.
Item #6 (measure the mount) should be done before trusting 6-DoF roll/pitch/z.

### Step 7 — Build, run, validate
```bash
cd ~/Workspaces/apriltag_ws
colcon build --packages-select apriltag_localization   # entry point unchanged; rebuild to install edits
source install/setup.bash
ros2 launch apriltag_localization localize.launch.py
ros2 topic echo /vessel_pose        # now PoseWithCovarianceStamped
```
**Validation (reuse existing rigs):**
- Static level bench: with the vessel/camera level, **roll ≈ pitch ≈ 0** and
  **z ≈ expected** (camera height chain). Large roll/pitch = mount error or a
  bug, not the tags.
- Re-run the **tags 1+3 fusion scenario** (already validated in 2D, see
  PHASE3_LOCALIZATION_RESULTS): x, y, yaw must match the old 2D result to confirm
  no regression, and z/roll/pitch must be sane.
- Sanity-check quaternion normalization (`‖q‖ = 1`) and that
  `frame_id`/`stamp` are populated (`ros2 topic echo`).

### Step 8 — Update docs & metadata
- `package.xml` description and `README.md` (currently say "Pose2D … x,y,theta").
- `docs/PHASE3_LOCALIZATION_RESULTS.md` (message-type line).
- PROJECT_STATE: mark Item #3, note the mount-coupling dependency on Item #6.

---

## 4. Gotchas
- **Quaternion double-cover** (`q` vs `-q`) in rotation averaging — normalize
  hemispheres or the mean collapses.
- **Timestamp**: must be the `/tags` header stamp, never `now()`, or the EKF
  fuses stale/misaligned poses.
- **frame_id agreement** with the EKF — a mismatch silently corrupts fusion.
- **Mount `T_B_C`** placeholder biases the new DoF directly (Item #6).
- **Don't regress the 2D path**: x/y/yaw must still match the prior validated
  numbers.

---

## 5. Suggested order of work
0. Confirm EKF contract (blocker) → 1. message+header+stamp → 2. keep full T_W_B
→ 3. rotation-aware fusion → 4. publish → 7. validate (level bench + 1+3 rerun)
→ 5. covariance → 8. docs. Do Item #6 (measure the mount) in parallel or right
after, since 6-DoF roll/pitch/z depend on it.

---

## 6. As-built (implemented 2026-07-21)

Everything in sections 1–5 above is the *plan*. This section records what was
actually built and how to run/validate it. Where the two differ, this section wins.

**Message type chosen: `geometry_msgs/PoseStamped`** (the section-2 fallback), not
`PoseWithCovarianceStamped`, because the downstream lidar+IMU EKF was specified to
consume `PoseStamped`. Consequence: **Step 5 (covariance) is dropped** — there is no
covariance field on the wire. Revisit if the EKF owner later wants weighting.

**`frame_id`** is a ROS parameter, default `"map"` — **still UNCONFIRMED with the EKF
owner**. A mismatch corrupts fusion silently (no error). Confirm before integration.

### 6a. How to launch

Already built; no rebuild needed unless you edit code. Three terminals, each must
`source install/setup.bash` first.

```bash
# Terminal 1 — camera + detector (produces /tags and /camera/camera_info)
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 launch apriltag_detector detect.launch.py resolution:=480p

# Terminal 2 — the localizer (prints per-tag + FUSED debug ~1 Hz)
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 launch apriltag_localization localize.launch.py

# Terminal 3 — watch the published 6-DoF pose
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 topic echo /vessel_pose
```

To override the world frame at launch:
`ros2 launch apriltag_localization localize.launch.py` then set the param, or run the
node directly with `--ros-args -p frame_id:=tank`.

### 6b. What to expect

`ros2 topic echo /vessel_pose` (Terminal 3) is the real 6-DoF view:
```yaml
header:
  stamp: {sec: ..., nanosec: ...}    # must be NON-zero, near image-capture time
  frame_id: map
pose:
  position:    {x: ..., y: ..., z: ...}          # all three now (z was discarded before)
  orientation: {x: ..., y: ..., z: ..., w: ...}  # the quaternion
```

**Console asymmetry (by design):** the `FUSION` + `FUSED` debug lines in Terminal 2
only print when **>=2 tags are visible** (the `len(estimates) >= 2` gate). The `FUSED`
line is where the fused `x y z` and full quaternion `q=[...]` show up. With a **single
tag**, Terminal 2 stays quiet but Terminal 3 **still publishes** — do not read console
silence as "not working"; check the topic.

### 6c. Runtime validation checklist

1. **`stamp` is non-zero** — the single most important runtime check (the bench test
   can't cover it). If `sec`/`nanosec` are `0`, `/tags` isn't carrying a header stamp
   upstream and the image-time copy is a no-op; the EKF would reject/misplace every
   pose. Non-zero and tracking wall-clock = the stamp path works.
2. **Baseline orientation**: with the vessel facing the world reference, `w`≈1 and
   `x,y,z`≈0 (i.e. ~zero rotation).
3. **No 2D regression**: rerun the **tags 1+3** scenario; x/y/yaw must match the prior
   validated 2D numbers (see PHASE3_LOCALIZATION_RESULTS).
4. **Level bench**: with vessel + camera level, roll≈pitch≈0 and z≈expected. A constant
   roll/pitch offset here is the `T_B_C` mount placeholder (Item #6), **not** this code.

### 6d. Reading the quaternion

Raw `x,y,z,w` isn't human-readable. For the level-bench check (roll≈pitch≈0) it's far
easier to convert to Euler degrees. Either paste the four numbers for a one-off
conversion, or write a throwaway subscriber (in the style of `range_probe.py` at the
workspace root) that prints live roll/pitch/yaw.

### 6e. If Terminal 3 prints nothing

First suspect is `/tags`, not the localizer. Check detections are actually flowing:
```bash
ros2 topic echo /tags        # confirm detections are non-empty
```
Also confirm the visible tag IDs exist in `config/tag_map.yaml` — unmapped IDs are
skipped, and weak detections (`decision_margin < 30`) are rejected.

### 6f. Replaying a rosbag (offline testing, with the detector overlay)

A recorded bag is the best way to exercise the localizer repeatably — no camera, no
tank, same input every run. Bags recorded with `apriltag_detector`'s
`record.launch.py` contain `/tags` and `/camera/camera_info`, which is **everything
`vessel_localizer` subscribes to**, so the localizer runs straight off the bag.

**Do NOT launch `detect.launch.py` alongside a bag.** It starts `gscam` (the physical
camera) and a second detector whose `/tags` would collide with the bag's. To see the
annotated tag overlay you only want the **`apriltag_draw`** node.

Two non-obvious requirements:
- Bags store `/camera/image_raw/compressed` only. `apriltag_draw` wants raw, so an
  `image_transport republish` is needed in between. (`draw.launch.py
  image_transport:=compressed` **crashes** with `std::out_of_range` — that launch
  file's remap pattern only works for raw transport. Republish instead.)
- `draw.launch.py` puts the node in a `camera` namespace, so the overlay publishes on
  **`/camera/image_tags`**, not `/image_tags` (the name `detect.launch.py` uses).

```bash
# ── Terminal 1: compressed -> raw (bags carry no raw images)
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 run image_transport republish --ros-args \
  -p in_transport:=compressed -p out_transport:=raw \
  -r in/compressed:=/camera/image_raw/compressed \
  -r out:=/camera/image_raw

# ── Terminal 2: overlay node (draws tag outlines)
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 launch apriltag_draw draw.launch.py tags:=/tags   # leading slash REQUIRED

# ── Terminal 3: the localizer (prints the 6-DoF FUSION/FUSED lines)
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 launch apriltag_localization localize.launch.py

# ── Terminal 4: view the overlay
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/image_tags

# ── Terminal 5: play the bag — START THIS LAST
cd ~/Workspaces/apriltag_ws && source install/setup.bash
ros2 bag play ~/experiment_bags/<bag_dir>
```

- **`tags:=/tags` needs the leading slash** — without it the node looks for
  `/camera/tags` (namespaced) and never sees the bag's `/tags`.
- **Start the bag last.** If tags arrive before `apriltag_draw` has images to match
  them against, you get `tag buffer queue overrun: 200` and no overlay.
- `--rate 0.5` to study individual frames, `--rate 5` to skim.

#### DO NOT use `ros2 bag play --loop` with apriltag_draw

**`apriltag_draw` dies permanently at the first loop boundary.** Measured: overlay
frames climb to ~1655 (one full pass), then flatline forever while the console floods
with `image buffer queue overrun: 200`.

Cause: `apriltag_draw` matches each image to the `/tags` message with the **same header
stamp**, and assumes stamps move forward. `--loop` jumps timestamps ~66 s into the
past, so every subsequent message looks stale and matching never resumes. **Queue depth
is not the lever** — tested at `max_queue_size` 200 and 10; both flatline at the same
point (10 is merely noisier). Play once, then re-run.

For continuous replay, restart the draw node each pass (Terminal 2):
```bash
BAG=~/experiment_bags/<bag_dir>
while true; do
  ros2 run apriltag_draw apriltag_draw_node --ros-args \
    -r __ns:=/camera -p image_transport:=raw -p max_queue_size:=200 \
    -r image:=image_raw -r tags:=/tags > /dev/null 2>&1 &
  DRAW=$!
  sleep 2
  ros2 bag play "$BAG"          # no --loop
  kill $DRAW; wait $DRAW 2>/dev/null
done
```
The republish node and `vessel_localizer` both survive the time jump — only
`apriltag_draw` needs restarting.

**`vessel_localizer` is immune to this by design:** every `/tags` message is converted,
fused and published on its own, with no cross-message buffering or time watermark, so
there is no state to get stuck. You can loop a bag against the localizer indefinitely —
useful for an offline regression harness (e.g. the tags 1+3 check).

**Note on frame counts:** in the 2026-07-17 tank bag the camera ran 30 Hz (1996 images)
while `/tags` came at 25 Hz (1661) — the detector dropped ~17% of frames at 480p.
Harmless for localization (no need for every frame), but it is why ~335 images never
find a matching tag and slowly fill `apriltag_draw`'s queue even without looping.

### 6g. Bench status at implementation time
Builds clean; node advertises `/vessel_pose` as `geometry_msgs/msg/PoseStamped`;
`frame_id` resolves to `"map"`; quaternion math verified numerically (R→q→R round-trip
1.1e-15, all four Shepperd branches exact at 180°, double-cover invariance exact to 0
bits). Items 1–4 of the 6c checklist remain — they need the camera.
