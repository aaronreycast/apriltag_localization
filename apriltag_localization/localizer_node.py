import math
import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose2D # simple X,Y,theta message


def object_points(size):
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


class VesselLocalizer(Node):
    def __init__(self):
        super().__init__('vessel_localizer')

        # # --- Parameters ---
        self.declare_parameter("tag_map_path", "") #path to tag_map.yaml
        map_path = self.get_parameter("tag_map_path").value
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
        self.pub = self.create_publisher(Pose2D, '/vessel_pose', 10) #I'm still figuring out what to publish here, but at least X,Y and theta
        self.get_logger().info("vessel_localizer node initialized")
    
    def on_camera_info(self, msg: CameraInfo):
        # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = np.array(msg.k, dtype=np.float64).reshape(3,3)
        # D = [k1, k2, p1, p2, k3]
        self.D = np.array(msg.d, dtype=np.float64)
    
    # Pending to create on_tags, load_tag_map, make_camera_mount function
    
    def load_tag_map(self, path):
        """ """
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

            x = T_W_B[0, 3]
            y = T_W_B[1, 3]
            yaw = math.atan2(T_W_B[1,0], T_W_B[0,0]) # heading in world XY
            estimates.append((x,y,yaw))
            weights.append(det.decision_margin) # trust stronger detections more
            tag_ids.append(det.id)

        if not estimates:
            return

        # Fusion-validation logging: with >=2 tags, print each tag's independent
        # estimate + weight so you can confirm they agree and see how fuse() combines
        # them. Throttled to 1 Hz to keep the console readable at sensor rate.
        if len(estimates) >= 2:
            lines = [
                f"  tag {tid:2d}: x={e[0]:+.3f} y={e[1]:+.3f} yaw={math.degrees(e[2]):+7.2f} deg  w={w:.1f}"
                for tid, e, w in zip(tag_ids, estimates, weights)
            ]
            self.get_logger().info(
                f"FUSION: {len(estimates)} tags visible\n" + "\n".join(lines),
                throttle_duration_sec=1.0)

        x,y,yaw = self.fuse(estimates, weights)
        if len(estimates) >= 2:
            self.get_logger().info(
                f"  FUSED : x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+7.2f} deg",
                throttle_duration_sec=1.0)
        out = Pose2D()
        out.x, out.y, out.theta = float(x), float(y), float(yaw)
        self.pub.publish(out)
        
    def fuse(self, estimates, weights):
        """Robust fusion when several tags are detected simultaneously"""
        w = np.array(weights, dtype=np.float64)
        w /= w.sum()
        xs = np.array([e[0] for e in estimates])
        ys = np.array([e[1] for e in estimates])
        # average yaw via unit vectors (handles the +/-180 deg wrap correctly)
        cs = np.array([math.cos(e[2]) for e in estimates])
        sn = np.array([math.sin(e[2]) for e in estimates])
        x = float(np.dot(w, xs))
        y = float(np.dot(w, ys))
        yaw = math.atan2(float(np.dot(w,sn)), float(np.dot(w, cs)))
        return x,y,yaw
        

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
    