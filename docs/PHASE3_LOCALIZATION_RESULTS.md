# Phase 3 — 2D Vessel Localization Node: COMPLETE (one recal fix pending), 2026-07-07

1. Deliverable built: `apriltag_localization` (ament_python) package with node `vessel_localizer`
   (`localizer_node.py`). Subscribes `/tags` (AprilTagDetectionArray) + `/camera/camera_info`,
   runs OpenCV solvePnP per detected tag, and publishes the vessel pose on `/vessel_pose`.
   (Originally `geometry_msgs/Pose2D`: x, y in metres, theta in RADIANS. **Superseded
   2026-07-21 by Open Item #3** — now `geometry_msgs/PoseStamped`, full 6-DoF with header;
   the 2D results below remain valid for the x/y/yaw components.)

2. Transform chain (world <- tag <- camera <- boat): T_W_B = T_W_T · inv(T_C_T) · inv(T_B_C).
   - T_C_T  : tag pose in camera frame, from solvePnP (corners + tag size + intrinsics).
   - T_W_T  : tag pose in world, from the tag-map YAML.
   - T_B_C  : camera pose on the boat (measured mount).
   Output read-off: X = T_W_B[0,3], Y = T_W_B[1,3], yaw = atan2(T_W_B[1,0], T_W_B[0,0]).

3. World frame (tank, top view): origin at top-left corner, X points DOWN, Y points RIGHT,
   Z up (water surface = z=0). Tags mounted 0.5 m above the water. tag_map.yaml holds 11 tags
   (IDs 0-10); all theta_deg values verified against the physical layout (top wall theta=0,
   left wall theta=90, right wall theta=-90 — normals point into the tank).

4. Multi-tag fusion: when several mapped tags are visible, poses are combined by a
   decision_margin-weighted average (yaw averaged via unit vectors to handle +/-180 wrap).
   Weak detections (decision_margin < 30) and unmapped IDs are rejected.

5. KEY FIX (corrects the beginner guide): cv2.SOLVEPNP_IPPE_SQUARE needs object/image points in
   TL,TR,BR,BL order. The /tags message delivers corners as BL,BR,TR,TL, so they must be
   REVERSED. Feeding message order directly returned a DEGENERATE pose (tvec Z ~ 0, reprojection
   error ~1e9 px) while Y/yaw still looked plausible — a silent trap. Reordering both the object
   points and the image points fixed it (reprojection error 0.09 px).

6. Validation (tag 0, head-on): with correct intrinsics tvec Z tracks real distance; the full
   chain reproduces the expected vessel pose. Example at ~1 m: X ~ 1.2 m (range + 0.2 m mount),
   Y ~ 2.45 m, yaw ~ 180 deg (boat faces the wall tag — 180 deg is correct, not 0).

7. Build/run gotchas resolved this phase:
   - setup.py listed launch/localize.launch.py in data_files but the file was missing -> colcon
     "can't copy" error. Created the launch file.
   - Detector's camera_480p.yaml existed in src but only camera_1080p.yaml was installed (stale
     install) -> gscam fell back to 1080p intrinsics for 480p images. Rebuilt apriltag_detector.
   - clean Ctrl-C shutdown (guarded double rclpy.shutdown()); resolution arg must be "480p".

8. RANGE-SCALE FINDING  [RESOLVED 2026-07-09 — see closing note below]: raw solvePnP range
   reads ~7.2% long.
   tvec_Z vs tape-measured distance, tag head-on:

   tape (m)   tvec_Z (m)   Z - tape    Z / tape
   0.50       0.535        +0.035      1.071
   1.00       1.076        +0.076      1.076
   1.50       1.606        +0.106      1.070

   Least-squares fit: tvec_Z = 1.072 * tape + 0.0004. Constant ratio + ~0 intercept = a pure
   MULTIPLICATIVE scale error (not a reference-point offset). Tag size ruled out (135.3 mm black
   edge, correct tag36h11 convention), so the cause is the 480p focal length: fx ~ 456.8 is ~7%
   too high (implies ~70 deg HFOV; true is ~426 / ~74 deg). The localization node itself is
   correct — the fix is a camera RECALIBRATION at 480p with deliberate depth + tilt + frame-corner
   coverage (low reprojection error alone did not guarantee correct fx). Deferred to a later
   session. Re-verify afterward: repeat this 3-distance test; the ratio should collapse to ~1.00.

9. Decision: keep running at 480p (per Phase 2). Node is production-ready pending the recal;
   until then, treat published ranges as ~7% long.

   CLOSING NOTE (2026-07-09): finding 8 was correct and the fix landed. The 480p fx/fy were
   metric-rescaled to 424.29 / 424.67 (cx/cy and distortion kept) and re-verified with the
   same 3-distance probe: slope 1.072 -> 1.001, intercept +0.011 m (a benign lens-surface-to-
   pinhole reference offset, not a scale term). Published 480p ranges are no longer long.
   Note the diagnosis in item 8 above understated one thing: the checkerboard solver was not
   merely "imprecise", it trades focal length against distortion, and low reprojection error
   does not constrain that trade at all. A physical range test is the source of truth for
   fx/fy on this lens -- now written up as CAMERA_CALIBRATION.md section 5c.

Still open before Phase 3 is fully closed:
- [DONE 2026-07-09] Recalibrate the 480p intrinsics (removed the 7.2% range scale error).
- Measure the real camera mount t_B_C (forward x, lateral y) on the boat and update
  make_camera_mount() — currently z=0.5 measured, x/y are placeholders that bias X/Y.
  This is now the LARGEST unquantified term in the pose budget: a 0.20 m placeholder against
  a ~0.5% (~20 mm at 5 m) residual calibration error.
- [ANSWERED 2026-07-28] "re-check whether 720p / 1080p share the same scale error" — YES.
  1080p read +7.9% long, the same disease from the same cause; fixed by metric rescale to
  fx 948.36 / fy 946.28, verified slope 1.005 (per-station error 8.8/8.1/8.0/8.2/7.9% ->
  0.4/0.2/0.33/0.45/0.44%). 720p was measured the same day and erred in the OPPOSITE
  direction -- 2.5% SHORT -- rescaled to fx 632.88 / fy 631.77, verified slope 1.0002
  (per-station error -2.6/-2.3/-2.5/-2.4/-2.6% -> +1.0/+0.4/+0.27/+0.35/+0.16%). All three
  resolutions are now metric-verified. Cross-check: 720p fx * 1.5 = 949.32 vs 1080p's
  independently measured 948.36 (0.10% apart), confirming 720p is a true 2/3 downscale.

10. MULTI-TAG FUSION VALIDATED (2026-07-07, live, 480p): two coplanar tags on the left wall,
    tag 1 (2.5,0) + tag 3 (5.0,0), camera hand-held at the midpoint 2.45 m normal to the wall.
    Eval point (camera): world (3.75, 2.45), boat facing -Y -> expected pose (3.75, 2.45, -90 deg).
    Added per-tag debug logging in on_tags (throttled 1 Hz, logs each tag's independent
    x/y/yaw/weight + the FUSED result) to make fusion observable.

    tag 1: x~3.93  y~2.79  yaw~-93.9 deg  w~171
    tag 3: x~3.81  y~2.79  yaw~-90.8 deg  w~185   (higher decision_margin)
    FUSED: x~3.87  y~2.79  yaw~-92.4 deg          (lands between the two, leans to tag 3)

    - Fusion PASSES: the two tags independently agree (~12 cm in X, ~3 deg in yaw, Y identical)
      despite being viewed ~27 deg obliquely, and the decision_margin weighting pulls FUSED
      toward the stronger tag 3. No yaw-wrap branch exercised (both near -90 deg).
    - Absolute offsets vs expectation match the two PENDING fixes quantitatively:
        Y +0.34 m = +7.2% range scale on 2.45 m (~+0.18) + placeholder mount x=0.20 along -Y
          (~+0.20) -> predicted ~2.83 vs measured 2.79. Fully explained.
        X +0.12 m, yaw -2.4 deg = small hand-held centering/tilt. (Pure scale error would push
          the two tags' X in OPPOSITE directions and cancel in the average; both read slightly
          high, so this ~0.15 m is placement, not the model.)
    - Caveat: this pair is coplanar, so it validates fusion + same-wall consistency but NOT
      cross-wall map consistency. Follow-up: perpendicular pair (e.g. tag 1 + tag 0, or 1 + 2).
    - Unrelated: rqt_image_view crashes on the compressed topic ("invalid allocator") with
      visualise:=true; harmless to detection/localization. Drop the arg or view /image_tags raw.

Launch used: ros2 launch apriltag_localization localize.launch.py
