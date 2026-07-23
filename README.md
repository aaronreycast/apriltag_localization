# apriltag_localization

2D pose estimation of a surface vessel (ASV) inside a tank, from AprilTags mounted on
the tank walls. A ROS 2 (Jazzy, `ament_python`) package providing the **`vessel_localizer`**
node: it consumes AprilTag detections, solves each tag's pose with OpenCV `solvePnP`,
chains the transforms through a known tag map, and publishes the vessel's `(x, y, yaw)`.

This package does **pose estimation**, not detection. It expects tag detections on `/tags`
(e.g. from [`apriltag_detector`](https://github.com/aaronreycast/apriltag_detector)) plus the
camera intrinsics on `/camera/camera_info`.

## How it works

For every detected tag that exists in the map, the node builds the chain
(world ← tag ← camera ← boat):

```
T_W_B = T_W_T · inv(T_C_T) · inv(T_B_C)
```

| Transform | Meaning | Source |
|-----------|---------|--------|
| `T_C_T` | tag pose in the camera frame | `solvePnP` (corners + tag size + intrinsics) |
| `T_W_T` | tag pose in the world | `tag_map.yaml` |
| `T_B_C` | camera pose on the boat (mount) | measured, `make_camera_mount()` |

The vessel pose is read off as `X = T_W_B[0,3]`, `Y = T_W_B[1,3]`,
`yaw = atan2(T_W_B[1,0], T_W_B[0,0])`.

**Multi-tag fusion:** when several mapped tags are visible in one frame, their independent
pose estimates are combined by a `decision_margin`-weighted average (yaw is averaged via unit
vectors to handle the ±180° wrap). Weak detections (`decision_margin < 30`) and unmapped tag
IDs are rejected.

### World frame

Tank top-view: origin at the top-left corner, **X points down, Y points right, Z up**
(water surface = `z = 0`). Tags are mounted upright on the walls; each map entry's
`theta_deg` is the wall-normal heading (top wall `0°`, left wall `90°`, right wall `-90°`).

## Node: `vessel_localizer`

| | Topic | Type |
|---|-------|------|
| **Subscribes** | `/tags` | `apriltag_msgs/AprilTagDetectionArray` |
| **Subscribes** | `/camera/camera_info` | `sensor_msgs/CameraInfo` |
| **Publishes** | `/vessel_pose` | `geometry_msgs/PoseStamped` — full 6-DoF: `pose.position` (x, y, z in metres) + `pose.orientation` (unit quaternion). `header.stamp` is the **image-acquisition time** (copied from `/tags`, not publish time); `header.frame_id` is the `frame_id` parameter. |

**Parameters**

| Name | Default | Description |
|------|---------|-------------|
| `tag_map_path` | `config/tag_map.yaml` (via launch) | Absolute path to the tag-map YAML |
| `frame_id` | `map` | World frame stamped into `header.frame_id` of the published pose. **Must match what the downstream EKF expects** — a mismatch corrupts fusion silently. |

## Tag map format

`config/tag_map.yaml` — positions in metres, angles in degrees:

```yaml
default_tag_size: 0.135      # black-square edge length (m)
tags:
  0:
    x: 0.0
    y: 2.45
    z: 0.5                   # tag-center height above the water surface
    theta_deg: 0.0           # wall-normal heading
    # size: 0.135            # optional, overrides default_tag_size
  1:
    x: 2.5
    y: 0.0
    z: 0.5
    theta_deg: 90.0
```

## Build

```bash
cd ~/Workspaces/apriltag_ws
colcon build --packages-select apriltag_localization --symlink-install
source install/setup.bash
```

Requires `apriltag_msgs` (from the detector stack), `rclpy`, `sensor_msgs`, `geometry_msgs`,
and OpenCV (`cv2`) + `numpy` + `pyyaml` in the Python environment.

## Run

Start a detector publishing `/tags` and `/camera/camera_info` (see `apriltag_detector`), then:

```bash
ros2 launch apriltag_localization localize.launch.py
# custom map:
ros2 launch apriltag_localization localize.launch.py tag_map_path:=/abs/path/to/tag_map.yaml
```

Inspect the output:

```bash
ros2 topic echo /vessel_pose
```

When two or more mapped tags are visible, the node also logs each tag's independent
estimate and the fused result (throttled to 1 Hz) for validation.

## Status & known limitations

The node is validated end-to-end (single tag, and multi-tag fusion with a coplanar pair).
See [`docs/PHASE3_LOCALIZATION_RESULTS.md`](docs/PHASE3_LOCALIZATION_RESULTS.md) for the full log.

The former **~+7.2% range-scale error is fixed.** The 480p intrinsics were rescaled via a
physical range test (`fx → 424.29`, `fy → 424.67`, `cx/cy`/distortion kept), which is the
source of truth for this wide/distorted lens rather than the unstable checkerboard fit. A
3-distance re-probe (0.50→0.510, 1.00→1.014, 1.50→1.511) refits to **slope 1.001** (was 1.072);
the residual is a benign ~1 cm constant offset (lens-surface-to-pinhole reference). Ranges are
now metric.

One calibration item still biases the absolute pose:

- **Camera mount `t_B_C`** — in `make_camera_mount()`, `z = 0.5` m is measured but the forward
  (`x`) and lateral (`y`) offsets are still placeholders (`[0.20, 0.0]`) and directly bias the
  published X/Y. Measure them on the real boat and update.

An implementation note worth flagging: `cv2.SOLVEPNP_IPPE_SQUARE` requires object/image points
in `TL, TR, BR, BL` order, whereas the `/tags` message delivers corners as `BL, BR, TR, TL` —
the node reverses them. Feeding the message order directly yields a *degenerate* pose (tvec
Z ≈ 0, huge reprojection error) while Y/yaw still look plausible — a silent trap.

## License

This software is issued under the Apache License, Version 2.0.
