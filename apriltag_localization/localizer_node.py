import math
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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


class VesselLocalizer(Node):
    def __init__(self):
        super().__init__('vessel_localizer')

        # # --- Parameters ---
        self.declare_parameter("tag_map_path", "") #path to tag_map.yaml
        map_path = self.get_parameter("tag_map_path").value
        self.declare_parameter("frame_id","map")
        self.frame_id = self.get_parameter("frame_id").value
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
        """Camera looking straight forward. Columns = optical axes in base coordinates:
        optical X (right) = base -Y
        optical Y (down) = base -Z
        optical Z (forward) = base +X   
        """
        # Rotation part
        R_B_C = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype = np.float64)
        # Translation part (where the camera lens sits on the boat, in base frame, meters).
        #   z = 0.5  -> camera is 0.5 m above base_link (measured, see mount sketch).
        #   x, y     -> forward / left offset of the lens from base_link. THESE BIAS X/Y
        #               DIRECTLY -> measure them on the real boat and replace the guesses.
        t_B_C = [0.20, 0.0, 0.5] # x(fwd, TODO measure), y(left, TODO measure), z(up)=0.5
        return make_T(R_B_C, t_B_C)
    
    def tag_pose_in_camera(self, det):
        """Estimate one tag's pose in the camera frame from its detected corners.

        Runs cv2.solvePnP (IPPE_SQUARE) on the tag's object points and the message
        corners reversed to TL,TR,BR,BL. Returns the 4x4 transform T_C_T, or None if
        solvePnP fails.
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

        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE) #best for planar square (4 coplanar points)
        if not ok:
            return None
        
        R, _ = cv2.Rodrigues(rvec) #3x3 rotation matrix

        T_C_T = np.eye(4)
        T_C_T[:3,:3] = R
        T_C_T[:3,3] = tvec.flatten()
        return T_C_T # tag expressed in camera frame (4x4 homogeneous transform)
    
    def on_tags(self, msg: AprilTagDetectionArray):
        """Subscription callback for /tags: turn detected tags into a vessel pose.

        For each detection, run the transform chain
        T_W_B = T_W_T @ inv(T_C_T) @ inv(T_B_C) to get one full 6-DoF estimate of
        the vessel in the world, skipping unknown IDs and weak detections
        (decision_margin < 30). The estimates are fused (weighted by decision_margin)
        into a single PoseStamped published on /vessel_pose, stamped with the
        incoming image-acquisition time and tagged with the configured frame_id.
        When >=2 tags are visible, each tag's independent estimate and the fused
        result are logged at 1 Hz so their agreement can be checked. Returns early
        without publishing if intrinsics have not arrived yet or if no tag produced
        a valid estimate.

        NOTE: reported z/roll/pitch are only as good as the camera mount T_B_C,
        whose x/y translation is still a placeholder (see Open Item #6).
        """
        if self.K is None:
            return # no intrinsics yet
        estimates = []
        weights = []
        tag_ids = [] # parallel to estimates/weights, for fusion validation logging
        for det in msg.detections:
            if det.id not in self.tag_map:
                continue # unknown tag, skip
            if det.decision_margin < 30.0: # reject weak detections (tune threshold as needed
                continue

            T_C_T = self.tag_pose_in_camera(det)
            if T_C_T is None:
                continue # could not solvePnP for this tag
            
            T_W_T = self.tag_map[det.id][0]
            #THE CHAIN
            T_W_B = T_W_T @ invert_T(T_C_T) @ invert_T(self.T_B_C)

            estimates.append(T_W_B)
            weights.append(det.decision_margin) # trust stronger detections more
            tag_ids.append(det.id)

        if not estimates:
            return

        # Fusion-validation logging: with >=2 tags, print each tag's independent
        # estimate + weight so you can confirm they agree and see how fuse() combines
        # them. Throttled to 1 Hz to keep the console readable at sensor rate.
        if len(estimates) >= 2:
            lines = [ ]
            for tid, T, w in zip(tag_ids, estimates, weights):
                x, y, z, roll, pitch, yaw = xyz_rpy(T)
                lines.append(
                    f"  tag {tid:2d}: x={x:+.3f} y={y:+.3f} z={z:+.3f}"
                    f" roll={math.degrees(roll):+7.2f} pitch={math.degrees(pitch):+7.2f} yaw={math.degrees(yaw):+7.2f} deg  w={w:.1f}")

            self.get_logger().info(
                f"FUSION: {len(estimates)} tags visible\n" + "\n".join(lines),
                throttle_duration_sec=1.0)
            
        t, q = self.fuse(estimates, weights)

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
        
    def fuse(self, estimates, weights):
        """Combine several T_W_B estimates into one 6-DoF pose, weighted by `weights`.

        Translation is a weighted average; rotation uses Markley quaternion averaging. 
        Returns (t,q), a 3 vector [x, y, z] and a quaternion [x, y, z, w]
        """
        w = np.array(weights, dtype=np.float64)
        w /= w.sum()
        ts = np.array([T[:3,3] for T in estimates]) # shape (N,3): one row per tag
        t = w @ ts # shape (3,): weighted average ('@' matrix multiplication)
        quats = [rot_to_quat(T[:3,:3]) for T in estimates]
        q = average_quats(quats, w)
        return t, q

def main(args=None):
    rclpy.init(args=args)
    node = VesselLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy may already be shut down (e.g. Ctrl-C); guard against double shutdown.
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    main()
    