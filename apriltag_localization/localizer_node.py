import math
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from collections import namedtuple

from rcl_interfaces.msg import SetParametersResult

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped # adds a Header (timestamp+frame_id) + a Pose(a 3D position and an orientation quaternion)= 6 DoF + timing


def object_points(size):
    """Return the 4 tag corners in the tag's own frame (meters), for solvePnP.

    Points are centered at the tag origin with half-edge h = size/2, in the
    TL,TR,BR,BL order that cv2.SOLVEPNP_IPPE_SQUARE requires.
    """
    h = size / 2.0
    # cv2.SOLVEPNP_IPPE_SQUARE requires this EXACT order (OpenCV convention, +Y up):
    # TL, TR, BR, BL. The image points are reordered to match (see tag_pose_in_camera).
    # NOTE: the /tags message corner order is BL,BR,TR,TL -> NOT the same, so it must
    # be reversed. Feeding message order directly to IPPE_SQUARE returns a degenerate
    # pose (Z~0, huge reprojection error).
    return np.array([
        [-h, h, 0],  # top-left
        [h, h, 0],   # top-right
        [h, -h, 0],  # bottom-right
        [-h, -h, 0], # bottom-left
    ], dtype=np.float64)
    
# Transform helpers
def make_T(R, t):
    """Make a 4x4 homogeneous transform from rotation matrix and translation vector."""
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = np.asarray(t, dtype=np.float64).flatten()
    return T

def invert_T(T):
    """Fast inverse of a rigid transform: R^T and -R^T t """
    R = T[:3,:3]
    t = T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3,3] = -R.T @ t
    return Ti

def tag_world_T(x,y,z, theta_deg):
    """Tag pose in world from a map entry (upright wall tag)"""
    th = math.radians(theta_deg)
    c, s = math.cos(th), math.sin(th)
    R_W_T = np.array([
        [-s, 0, c],
        [c, 0, s],
        [0, 1, 0]
    ],
    dtype = np.float64)
    return make_T(R_W_T, [x, y, z])

def rot_to_quat(R):
    """Convert a 3x3 rotation matrix to a quaternion [x, y, z, w].

    Shepperd's method: four branches anchored on w/x/y/z, choosing whichever
    component is largest so the divisor is never near zero (a single-branch
    formula blows up at theta = 180 deg, where w = cos(theta/2) -> 0).
    Implemented locally to avoid a scipy dependency on the Jetson.
    """
    trace = R[0,0] + R[1,1] + R[2,2]
    R_max = max(R[0,0], R[1,1], R[2,2])

    if trace > 0:
        S = 2*np.sqrt(1 + trace)          # = 4w
        w = S/4
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    elif R_max == R[0,0]:
        S = 2*np.sqrt(1 + R[0,0] - R[1,1] - R[2,2])     # = 4x
        w = (R[2,1] - R[1,2]) / S
        x = S/4
        y = (R[0,1] + R[1,0]) / S
        z = (R[0,2] + R[2,0]) / S
    elif R_max == R[1,1]:
        S = 2*np.sqrt(1 + R[1,1] - R[0,0] - R[2,2])     # = 4y
        w = (R[0,2] - R[2,0]) / S
        x = (R[0,1] + R[1,0]) / S
        y = S/4
        z = (R[1,2] + R[2,1]) / S
    else:
        S = 2*np.sqrt(1 + R[2,2] - R[0,0] - R[1,1])     # = 4z
        w = (R[1,0] - R[0,1]) / S
        x = (R[0,2] + R[2,0]) / S
        y = (R[1,2] + R[2,1]) / S
        z = S/4

    q = np.array([x,y,z,w], dtype=np.float64)
    return q / np.linalg.norm(q) # project back onto the unit sphere (float drift insurance)

def quat_to_rot(q):
    """Convert a quaternion [x, y, z, w] to a 3x3 rotation matrix.

    Inverse of rot_to_quat. Note the [x, y, z, w] order (geometry_msgs
    convention, NOT [w, x, y, z]) -- the same ordering trap as everywhere
    else in this file.

    NO BRANCHES NEEDED, unlike rot_to_quat. Shepperd's method has four cases
    because it DIVIDES by a quaternion component that can vanish; this
    direction only multiplies, so it is singularity-free and a case analysis
    here would be a bug, not caution.

    Every term is quadratic in q, so q and -q produce the IDENTICAL matrix --
    the double cover, for free. Same property verified numerically on the
    averaging side 2026-07-21 (avg(qa,qb) vs avg(qa,-qb) differ by 0 bits).

    ASSUMES q IS UNIT NORM and does not check. Safe for the two callers that
    exist: rot_to_quat normalizes on the way out, and average_quats returns an
    eigenvector from np.linalg.eigh, which is unit by construction. A
    hand-built quaternion is NOT, and would silently yield a matrix that is
    not a rotation (det != 1).

    WHY THIS EXISTS -- Two-pass fusion needs the fused
    orientation as a MATRIX, to rotate each tag's t_C_T into the world:
    R_W_C = R_W_B @ R_B_C, then p_W_C = p_W_T - R_W_C @ t_C_T. average_quats
    returns a quaternion, so this is the bridge.
    """
    x, y, z, w = q
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*w*y, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R

def quat_to_euler(q):
    """Convert a quaternion [x, y, z, w] to (roll, pitch, yaw) in radians.

    ZYX / aerospace convention (REP-103): yaw about Z, then pitch about Y, then
    roll about X. Chosen so the yaw returned here is identical to the
    atan2(R[1,0], R[0,0]) used throughout the 2D work -- switching the logs to
    full roll/pitch/yaw does not change any previously validated yaw number.

    DISPLAY ONLY. Euler angles are singular at pitch = +/-90 deg (gimbal lock:
    roll and yaw fold onto the same axis and become ambiguous), which is why the
    fusion math and the published pose both stay in quaternions. The |sinp| >= 1
    branch below clamps instead of letting float drift push asin() out of domain.
    """
    x, y, z, w = q

    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp) # gimbal-lock guard: clamp instead of NaN
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
    
def xyz_rpy(T):
    """Full 6-DoF read-out of a 4x4 transform: (x, y, z, roll, pitch, yaw).

    Position from the translation column; orientation via rot_to_quat ->
    quat_to_euler (ZYX). Angles in radians. Display/debug helper — the
    published pose keeps the quaternion, which has no gimbal-lock singularity.
    """
    roll, pitch, yaw = quat_to_euler(rot_to_quat(T[:3,:3]))
    return T[0,3], T[1,3], T[2,3], roll, pitch, yaw

def average_quats(quats, weights):
    """Weighted average of unit quaternions (Markely's method)
    Builds M = sum(w_i * q_i q_i^T) and takes the eigenvector of the largest
    eigenvalue. The outer product is identical for q and -q, so the quaternion
    double-cover cannot corrupt the average.
    """
    M = np.zeros((4,4))
    for q, wi in zip(quats, weights):
        M += wi * np.outer(q,q)
    eigvals, eigvecs = np.linalg.eigh(M) # symmetric -> real, ascending order
    q = eigvecs[:,-1]                    # last column = latest eigenvalue
    if q[3] < 0:                         # canonical hemisphere (w >= 0)
        q = -q
    return q / np.linalg.norm(q)


# One tag's contribution to a fused pose. A record rather than parallel lists so
# a `continue` in the detection loop can never desynchronize the fields.
#
# NOTE what is deliberately NOT in here -- Open Item #9. The per-tag ROTATION is
# absent by design. Position from one tag's corners is ~40x better determined
# than orientation from the same corners (20-41 deg/px at 2.0-2.85 m), so fusion
# derives every tag's position from the single FUSED rotation, never from that
# tag's own. `t_C_T` (tag in camera frame) and `p_W_T` (tag in world, from the
# map) are the two well-conditioned inputs that pass needs. `T_W_B` is kept only
# for the rotation average and the 1 Hz log. Do not add a per-tag rotation here:
# with it available, the contamination this record exists to prevent becomes one
# careless line away.
TagEstimate = namedtuple('TagEstimate', 'tag_id T_W_B weight ratio t_C_T p_W_T')

# Default for the min_pose_ratio parameter (Open Item #8). A module constant, not a
# literal in declare_parameter(), because OFFLINE TOOLS MUST BE ABLE TO IMPORT IT --
# tools/render_bag_demo.py reproduces the node's pipeline without constructing a Node,
# so a hard-coded 1.5 there would be a second copy of a tuned threshold, free to drift
# out of step with this one. Same failure that made the renderer draw one-pass fusion
# after the node moved to two-pass. See __init__ for how 1.5 was chosen.
DEFAULT_MIN_POSE_RATIO = 1.5


class VesselLocalizer(Node):
    def __init__(self):
        super().__init__('vessel_localizer')

        # # --- Parameters ---
        self.declare_parameter("tag_map_path", "") #path to tag_map.yaml
        map_path = self.get_parameter("tag_map_path").value
        self.declare_parameter("frame_id","map")
        self.frame_id = self.get_parameter("frame_id").value
        # Open Item #8 rejection gate. Minimum acceptable err[1]/err[0] from
        # solvePnPGeneric: how much worse the DISCARDED mirror solution fits than the
        # one being used. Near 1.0 the two reproject almost identically and the branch
        # was picked near-arbitrarily, so the published pose may be the mirrored one --
        # which barely moves the tag's translation but rotates it ~20 deg, and inv(T_C_T)
        # turns that into a position error of roughly range x angle.
        #
        # Default 1.5, chosen from a threshold sweep over two 2026-07-23 tank bags
        # (>0.5 m from own +/-1 s local median):
        #
        #   gate   pivot bag           aggressive bag
        #   none   100% kept / 19.9%   100% kept / 22.3%
        #   1.2     81% kept /  8.3%    76% kept /  9.5%
        #   1.5     70% kept /  6.9%    62% kept /  6.1%
        #   2.0     53% kept /  6.1%    49% kept /  5.6%
        #
        # The knee sits in the same place on both bags despite very different data;
        # past 1.5 you pay ~10% of your fixes per point of outlier. 1.2 is the
        # defensible alternative if rate matters more than purity to the consumer.
        # This is a SECONDARY absolute layer correcting a drifting INS, so trustworthy
        # fixes are worth more than frequent ones -- hence 1.5 rather than 1.2.
        #
        # VERIFIED NOT TO TOUCH CLEAN DATA: on the static two-tag bench fixture the
        # median ratio is 4.36 and 0 of 3752 detections fall below any gate up to 2.0.
        #
        # 0.0 DISABLES the gate, and needs no special case: `ratio` is a quotient of
        # non-negative reprojection errors, so it is always >= 0 (or inf), and
        # `ratio < 0.0` can never be true. Do not add a branch for it.
        self.declare_parameter("min_pose_ratio", DEFAULT_MIN_POSE_RATIO)
        self.min_pose_ratio = float(self.get_parameter("min_pose_ratio").value)

        # Cumulative from node start, for tuning min_pose_ratio on real data: if the
        # gate is throwing away most of what the detector finds, that is visible here
        # rather than only as a low pose rate. Cumulative means they must be read as a
        # rate over a whole run -- RESTART THE NODE between stations, same discipline
        # as edge_fov_probe_v2's counters.
        self.n_gated = 0 # detections rejected by the ambiguity gate
        self.n_kept = 0  # detections that passed it

        # min_pose_ratio is settable at RUNTIME, not just at launch. Without this the
        # value above is read once here and `ros2 param set` appears to succeed while
        # changing nothing -- the worst kind of knob. It matters because the only place
        # the threshold can honestly be chosen is on the water, against the real
        # geometry and the real consumer, and a relaunch there costs a station.
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.tag_map = self.load_tag_map(map_path) #{id: (T_W_T 4x4, size)}

        # # camera mount: vessel <- camera. Fill from your measured mount
        self.T_B_C = self.make_camera_mount()

        # intrinsics, filled once camera_info arrives
        self.K = None
        self.D = None

        # --- Subscriptions ---
        # camera_info is 'latching-ish' but just cache the latest one.
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info, 10)
        # tags come at sensor rate -> use sensor_data QoS to match the detector.
        self.create_subscription(
            AprilTagDetectionArray, '/tags', self.on_tags, qos_profile_sensor_data)
        
        # ---Publisher---
        self.pub = self.create_publisher(PoseStamped, '/vessel_pose', 10) #Publishing full pose estimate
        self.get_logger().info("vessel_localizer node initialized")

    def on_camera_info(self, msg: CameraInfo):
        """Cache the latest camera intrinsics: K (3x3) and distortion D."""
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        # D = [k1, k2, p1, p2, k3]
        self.D = np.array(msg.d, dtype=np.float64)
    
    def load_tag_map(self, path):
        """Load the tag map YAML into {id: (T_W_T 4x4, size)}.

        Each entry's (x, y, z, theta_deg) is turned into the tag's world pose via
        tag_world_T; the tag size falls back to the file's default_tag_size (0.135 m).
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        default_size = data.get('default_tag_size', 0.135)
        tag_map = {}
        for tag_id, entry in data["tags"].items():
            size = entry.get('size', default_size)
            T_W_T = tag_world_T(entry['x'], entry['y'], entry['z'], entry['theta_deg'])
            tag_map[int(tag_id)] = (T_W_T, size) #matches your {id: (T_W_T 4x4, size)}
        return tag_map
    
    # Camera Mount T_B_C (camera in boat frame transform)
    def make_camera_mount(self):
        """Camera pose on the vessel: T_B_C, camera frame expressed in base_link.

        Columns of R_B_C = the optical axes in base coordinates:
        optical X (right)   = base -Y
        optical Y (down)    = base -Z
        optical Z (forward) = base +X

        base_link origin = the TOP SURFACE OF THE REAR CENTRE CROSSBAR of the
        boat (decided 2026-08-06). That choice matters beyond this function: the
        pose published on /vessel_pose IS this point's pose, so it defines what
        "the boat is at (x, y)" means to everything downstream, including the EKF.

        ROTATION IS STILL ASSUMED IDENTITY -- perfectly forward-facing and level.
        That is unmeasured and is the remaining half of Open Item #6. Note it
        CANNOT be measured on a bench: the same camera read pitch +1.50 deg on
        2026-08-10 and +0.06 deg on 2026-08-11, so a bench reading measures the
        tripod head, not the mount. It has to be done on the boat, floating in
        normal loaded trim, from the optics (a tag at known height dead ahead
        projects to (cx, cy) only if the mount is perfect), and validated with the
        180 deg spin test.
        """
        # Rotation part
        R_B_C = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype = np.float64)

        # Translation part: where the camera lens sits on the boat, in base frame,
        # metres. Tape-measured on the boat 2026-08-07 from the crossbar datum:
        # 0.6259 m datum-to-LENS FACE, less the 4.5 mm lens-surface-to-pinhole
        # offset (the intercept `b` of the 720p range-test fit, CAMERA_CALIBRATION
        # section 5c). The correction is PER-RESOLUTION -- 4.5 mm at 720p, and it is
        # the weakest-determined term in that fit at roughly +/-5 mm session to
        # session, so do not read it as precise to the millimetre.
        #
        # Applied and validated 2026-08-11, replacing the [0.20, 0.0, 0.5]
        # placeholder that had dominated the pose error budget since 2026-07-17.
        # Validation was a before/after differential at one fixed station, which
        # makes it immune to the Open Item #9 bias: observed dy +417.1 mm against
        # +421.4 predicted, dz +448.9 vs +447.2, dx +2.4 vs +1.4, dyaw +0.026 deg
        # vs 0. Cross-checked two ways -- the camera's world z must be invariant
        # under a mount change and agreed to 1.7 mm, and the implied lens height
        # above the tag plane matched an independent probe measurement to 4 mm.
        t_B_C = [0.6259 - 0.0045, 0, 0.0528]
        return make_T(R_B_C, t_B_C)
    
    def tag_pose_in_camera(self, det):
        """Estimate one tag's pose in the camera frame from its detected corners.

        Runs cv2.solvePnPGeneric (IPPE_SQUARE) on the tag's object points and the
        message corners reversed to TL,TR,BR,BL. Returns (T_C_T, ratio), or
        (None, None) if the solve fails.

        WHY solvePnPGeneric AND NOT solvePnP -- Open Item #8. A square tag admits TWO
        poses that reproject almost identically, mirrored about the tag plane.
        solvePnP returns one of them with NO signal that the other was nearly as good.
        solvePnPGeneric returns both, plus both reprojection errors, so

            ratio = err[1] / err[0]

        measures how confidently the branch was picked: ratio >> 1 is a well-determined
        pose; ratio ~ 1 means the choice was near-arbitrary and the returned pose may be
        the mirrored one. On the 2026-07-23 pivot bag the median ratio was 2.01 with 33%
        below 1.5, and 18.5% of published fixes landed >0.5 m from their local median.

        The flip barely moves the tag's TRANSLATION (<1 mm) -- it rotates it by ~20 deg,
        and inv(T_C_T) in the chain turns that into a position error of roughly
        range x angle. So range checks look fine while the vessel pose swings metres.

        solutions[0] is BIT-IDENTICAL to what solvePnP returned (verified 2026-08-10 on
        real corners: max |diff| 0.0e+00 in both rvec and tvec), so swapping solvePnP for
        solvePnPGeneric did not by itself change any published pose.

        `ratio` is no longer diagnostic only: since 2026-08-12 on_tags REJECTS any
        detection with ratio < min_pose_ratio. Measured on the 07-23 tank bags, the gate
        cut the outlier rate 16.7% -> 3.0% (pivot) and 15.4% -> 2.3% (aggressive) for
        ~27% of fixes. This method itself still only reports -- it never rejects, so it
        stays reusable by offline tools that want the raw ratio distribution.
        """
        size = self.tag_map[det.id][1]
        obj = object_points(size)
        # message corners are BL,BR,TR,TL; reverse to TL,TR,BR,BL to match object_points
        # (the order cv2.SOLVEPNP_IPPE_SQUARE expects).
        mc = det.corners
        img = np.array([
            [mc[3].x, mc[3].y],  # top-left
            [mc[2].x, mc[2].y],  # top-right
            [mc[1].x, mc[1].y],  # bottom-right
            [mc[0].x, mc[0].y],  # bottom-left
        ], dtype=np.float64)

        n, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE) #planar square (4 coplanar points)
        if n < 1:
            return None, None

        # errs comes back shaped (n,1); ravel so the arithmetic below stays scalar
        # (float() on a >0-d array is deprecated in numpy >= 1.25).
        e = np.asarray(errs, dtype=np.float64).ravel()
        if n > 1 and e[0] > 0.0:
            ratio = float(e[1] / e[0])
        else:
            ratio = float('inf') # single solution, or an exact fit -> unambiguous

        # [0] is what solvePnP would have returned (verified bit-identical 2026-08-10).
        #
        # It is NOT necessarily the better-fitting solution. An earlier version of this
        # comment claimed the list is sorted by reprojection error -- MEASURED FALSE
        # 2026-08-12: across 1498 detections from two tank bags, 7 had ratio < 1.0
        # (minimum 0.8738), i.e. OpenCV handed back the WORSE-fitting branch first.
        # Those are the most suspect detections there are, and the min_pose_ratio gate
        # rejects all of them. solvePnPGeneric also returned exactly 2 solutions on
        # every one of those 1498 solves, so the inf branch above is defensive only.
        rvec, tvec = rvecs[0], tvecs[0]
        R, _ = cv2.Rodrigues(rvec) #3x3 rotation matrix

        T_C_T = np.eye(4)
        T_C_T[:3,:3] = R
        T_C_T[:3,3] = tvec.flatten()
        return T_C_T, ratio # tag in camera frame (4x4), + IPPE ambiguity ratio
    
    def on_set_parameters(self, params):
        """Validate and apply runtime parameter changes (`ros2 param set`).

        Registered in __init__. rclpy calls this BEFORE committing a change, with
        every parameter in the request; returning successful=False rejects the whole
        request and leaves the node untouched. So this is the only place that can
        refuse a bad value -- the gate's use site cannot, it just compares numbers.

        Only min_pose_ratio is handled. Anything else is accepted unchanged, because
        the remaining parameters (tag_map_path, frame_id) are read once at startup and
        silently ignoring a set on them would be the exact trap this method exists to
        remove; they are rejected explicitly instead.

        The threshold is the one number that can only honestly be chosen against real
        geometry, and on the water a relaunch costs a station. Sweep it live:

            ros2 param set /vessel_localizer min_pose_ratio 1.2

        Changing it RESETS the gate tally. gate_summary() reports a rate since node
        start; across a threshold change that rate is an average of two different
        gates, which is worse than useless because it still looks like a measurement.
        """
        for p in params:
            if p.name != "min_pose_ratio":
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} is read once at startup; set it at launch, not at runtime")

            # Reject rather than clamp. A negative gate is not a weaker gate -- ratio is
            # a quotient of non-negative errors, so ANY negative value disables the gate
            # entirely while reading in `ros2 param get` as if it were set. Use 0.0 for
            # that, which says so.
            if p.value < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f"min_pose_ratio must be >= 0.0 (got {p.value}); use 0.0 to disable the gate")

            old = self.min_pose_ratio
            self.min_pose_ratio = float(p.value)
            self.get_logger().info(
                f"min_pose_ratio {old} -> {self.min_pose_ratio} "
                f"(tally reset; previous: {self.gate_summary()})")
            self.n_gated = 0
            self.n_kept = 0

        return SetParametersResult(successful=True)

    def gate_summary(self):
        """One-line running tally of the Open Item #8 ambiguity gate, since node start.

        Reported alongside every log line the gate produces so the rejection rate is
        never something you have to go and measure separately. On the 2026-07-23 tank
        bags roughly 30-40% is normal at the 1.5 default; on the clean static bench
        fixture it is 0%. A rate far outside that band means the geometry changed, not
        that the threshold is wrong.
        """
        total = self.n_gated + self.n_kept
        pct = 100.0 * self.n_gated / total if total else 0.0
        return f"{self.n_gated}/{total} detections gated ({pct:.0f}%)"

    def on_tags(self, msg: AprilTagDetectionArray):
        """Subscription callback for /tags: turn detected tags into a vessel pose.

        For each detection, run the transform chain
        T_W_B = T_W_T @ inv(T_C_T) @ inv(T_B_C) to get one full 6-DoF estimate of
        the vessel in the world, skipping unknown IDs and weak detections
        (decision_margin < 30). Each surviving detection becomes one TagEstimate
        record; fuse() combines them into a single PoseStamped published on
        /vessel_pose, stamped with the incoming image-acquisition time and tagged
        with the configured frame_id. When >=2 tags are visible, each tag's
        independent estimate and the fused result are logged at 1 Hz so their
        agreement can be checked. Returns early without publishing if intrinsics
        have not arrived yet or if no tag produced a valid estimate.

        NOTE the per-tag T_W_B built here is NOT what determines the published
        position. Its rotation feeds the fused orientation and its full transform
        feeds the log, but fuse() re-derives every tag's position from the FUSED
        rotation -- see Open Item #9 and fuse()'s docstring for why using the
        per-tag one costs ~150 mm.

        NOTE the camera mount T_B_C is now measured (2026-08-11), so reported z is
        meaningful. Reported roll/pitch still inherit the mount's UNMEASURED
        rotation, which make_camera_mount() assumes is identity -- see Open Item #6.
        """
        if self.K is None:
            return # no intrinsics yet
        estimates = [] # list of TagEstimate, one per surviving detection
        n_gated = 0    # rejected by the Open Item #8 ambiguity gate, THIS frame
        for det in msg.detections:
            if det.id not in self.tag_map:
                continue # unknown tag, skip
            if det.decision_margin < 30.0: # reject weak detections (tune threshold as needed
                continue

            T_C_T, ratio = self.tag_pose_in_camera(det)
            if T_C_T is None:
                continue # could not solvePnP for this tag

            if ratio < self.min_pose_ratio:
                # Reject: the mirror solution fit nearly as well, so the branch was
                # picked near-arbitrarily. A dropped frame beats a fix that is metres
                # out with a clean decision_margin. Counted, never silent -- see below.
                n_gated += 1
                self.n_gated += 1
                continue

            self.n_kept += 1

            T_W_T = self.tag_map[det.id][0]
            #THE CHAIN
            T_W_B = T_W_T @ invert_T(T_C_T) @ invert_T(self.T_B_C)

            estimates.append(TagEstimate(
                tag_id=det.id,
                T_W_B=T_W_B,
                weight=det.decision_margin, # trust stronger detections more
                ratio=ratio,
                t_C_T=T_C_T[:3,3],          # tag in camera frame -- well conditioned
                p_W_T=T_W_T[:3,3]))         # tag in world, straight from the map

        if not estimates:
            # Distinguish "nothing was visible" from "everything visible was rejected".
            # Without this they look identical in the logs, and the second one is the
            # case that will confuse you on the water: tags plainly in frame, decoding
            # cleanly, and no pose coming out. Same failure of observability that made
            # a 3.6%-detection station indistinguishable from a healthy one in 2026-08-05.
            if n_gated:
                self.get_logger().warn(
                    f"no pose: all {n_gated} mapped detection(s) rejected as ambiguous "
                    f"(ratio < {self.min_pose_ratio}). Session: {self.gate_summary()}",
                    throttle_duration_sec=2.0)
            return

        # Fusion-validation logging: with >=2 tags, print each tag's independent
        # estimate + weight so you can confirm they agree and see how fuse() combines
        # them. Throttled to 1 Hz to keep the console readable at sensor rate.
        if len(estimates) >= 2:
            lines = [ ]
            for e in estimates:
                x, y, z, roll, pitch, yaw = xyz_rpy(e.T_W_B)
                lines.append(
                    f"  tag {e.tag_id:2d}: x={x:+.3f} y={y:+.3f} z={z:+.3f}"
                    f" roll={math.degrees(roll):+7.2f} pitch={math.degrees(pitch):+7.2f} yaw={math.degrees(yaw):+7.2f} deg  w={e.weight:.1f}"
                    f"  ratio={e.ratio:5.2f}") # Open Item #8: ~1 means the branch was a coin flip

            self.get_logger().info(
                f"FUSION: {len(estimates)} tags visible"
                + (f", {n_gated} gated" if n_gated else "")
                + f"  [{self.gate_summary()}]\n" + "\n".join(lines),
                throttle_duration_sec=1.0)
            
        t, q = self.fuse(estimates)

        if len(estimates) >=2:
            roll, pitch, yaw = quat_to_euler(q)
            self.get_logger().info(
                f"  FUSED : x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} "
                f"roll={math.degrees(roll):+7.2f} pitch={math.degrees(pitch):+7.2f} yaw={math.degrees(yaw):+7.2f} deg "
                f"q=[{q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f} {q[3]:+.3f}]",
                throttle_duration_sec=1.0
            )

        out = PoseStamped()
        out.header.stamp = msg.header.stamp # image_capture time, NOT now()
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(t[0])
        out.pose.position.y = float(t[1])
        out.pose.position.z = float(t[2])
        out.pose.orientation.x = float(q[0])
        out.pose.orientation.y = float(q[1])
        out.pose.orientation.z = float(q[2])
        out.pose.orientation.w = float(q[3])

        self.pub.publish(out)
        
    def fuse(self, estimates):
        """Combine several TagEstimate records into one 6-DoF pose.

        TWO PASSES, and the order is load-bearing -- Open Item #9. Pass 1 fuses
        the ROTATIONS (Markley quaternion averaging, weighted by decision_margin).
        Pass 2 then re-derives EVERY tag's position against that single fused
        rotation, and averages those.

        WHY NOT ONE PASS. Each tag's T_W_B translation column was built with THAT
        tag's own rotation (T_W_T @ inv(T_C_T) @ inv(T_B_C)), and orientation from
        one 0.135 m tag is ill-conditioned -- 20-41 deg/px at 2.0-2.85 m -- while
        position from the same corners is ~40x better determined. A per-tag yaw
        error dtheta on a tag at camera-frame lateral offset X displaces that tag's
        position by roughly X*dtheta. Two tags straddling the boresight have
        opposite X *and* opposite dtheta, so the two products SHARE a sign: the
        errors cancel in x and RECTIFY in y. Averaging cannot repair that, because
        the damage is done per-tag, upstream of the average.

        Measured 2026-08-11 on item9_fixture_20260811: per-tag yaw -95.45 and
        -84.82 deg -- 10.6 deg apart, yet their MEAN is right to 0.2 deg -- gave
        11 mm of x error and 147 mm of y error. Re-deriving the positions from the
        fused rotation takes the radial error from 157 mm to 26 mm.

        Pass 2 evaluates p_W_C = p_W_T - R_W_C @ t_C_T. That is the SAME identity
        the per-tag chain already evaluates (expand T_W_T @ inv(T_C_T) and it falls
        out); the only thing that changes is which rotation goes in.

        WITH A SINGLE TAG this reduces exactly to the old behaviour, since the
        fused rotation is then that tag's own rotation. A lone tag is still worth
        ~0.3 m of position error and needs a separate policy -- see Open Item #9.

        Returns (t,q), a 3 vector [x, y, z] and a quaternion [x, y, z, w]
        """
        w = np.array([e.weight for e in estimates], dtype=np.float64)
        w /= w.sum()

        # --- Pass 1: rotation. MUST come first; pass 2 consumes its output.
        quats = [rot_to_quat(e.T_W_B[:3,:3]) for e in estimates]
        q = average_quats(quats, w)
        R_W_B = quat_to_rot(q)
        R_W_C = R_W_B @ self.T_B_C[:3,:3] # camera orientation in world

        # --- Pass 2: position, every tag against that one rotation.
        p_cams = np.array([e.p_W_T - R_W_C @ e.t_C_T for e in estimates]) # (N,3)
        p_W_C = w @ p_cams # weighted average ('@' matrix multiplication)

        # The mount offset is common to every tag, so it factors out of the sum
        # and is applied once here instead of N times inside the comprehension.
        t = p_W_C - R_W_B @ self.T_B_C[:3,3]
        return t, q

def main(args=None):
    rclpy.init(args=args)
    node = VesselLocalizer()
    # Two DIFFERENT signals land here and both are clean stops -- exit 0 for each.
    # They are split rather than caught as one tuple so the supervisor case is
    # visible in the log: a node that is being restart-looped by systemd looks
    # identical to a clean run otherwise.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # SIGINT: a human pressed Ctrl-C, or `ros2 launch` forwarded it.
        pass
    except ExternalShutdownException:
        # SIGTERM: a supervisor stopped us -- launch tearing down after a sibling
        # died, systemd, `docker stop`, or a scripted replay harness. rclpy shuts
        # the context down externally, so spin() raises THIS, not KeyboardInterrupt.
        # Letting it escape exits 1 with a traceback and reports a false failure to
        # whatever is watching; on the Jetson that can drive a restart loop.
        # print(), not a logger, ON PURPOSE. By this point the rclpy context is torn
        # down, so ANY logger -- node.get_logger() or rclpy.logging.get_logger() --
        # still attempts a /rosout publish and rcl emits "Failed to publish log
        # message to rosout: publisher's context is invalid". That line looks like a
        # fault on what is a completely clean shutdown, which defeats the point of
        # logging here at all. stdout is captured by ros2 launch just the same.
        print('[vessel_localizer] stopping on external shutdown (SIGTERM)', flush=True)
    finally:
        node.destroy_node()
        # rclpy may already be shut down (e.g. Ctrl-C); guard against double shutdown.
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
    