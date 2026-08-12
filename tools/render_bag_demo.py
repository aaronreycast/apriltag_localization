#!/usr/bin/env python3
"""Render a recorded bag into a demo video: annotated camera view + top-down tank map.

This is the script that produced `docs/media/demo.gif` / `demo.mp4`. It is an offline
renderer, NOT a screen capture: it reads the bag directly, re-runs `vessel_localizer`'s own
maths over the recorded `/tags`, and composites each frame.

Why offline rather than replaying the pipeline and recording the screen:

  * it imports `localizer_node`'s helpers (`object_points`, `invert_T`, `tag_world_T`,
    `rot_to_quat`, `average_quats`, `quat_to_euler`, `make_T`) directly, so the pose on
    screen is produced by the shipped code, not a reimplementation;
  * it lets the intrinsics be chosen explicitly. A bag's own `/camera/camera_info` overwrites
    the localizer's cached K at camera rate, so bags recorded before the 2026-07-28 metric
    rescale drive the chain with `fx = 1022.34` instead of the verified `948.36` and produce
    poses that read ~8% long. Pass `--calib` and the bag's camera_info is ignored;
  * no rqt on the CPU, no desktop furniture in frame, and exact frame pacing.

Whatever is not measured is stated on the frame: the calibration file in use and the camera
mount currently compiled into the node.

Usage
-----
    python3 tools/render_bag_demo.py ~/experiment_bags/.../pivot_in_place_trial_02_1080p \\
        --calib ../apriltag_detector/apriltag_detector/config/camera_1080p.yaml \\
        --out demo.mp4 --start 30.4 --end 36.1

Then, for a README-sized GIF (about 4 MB for 6 s at 720 px wide):

    ffmpeg -i demo.mp4 -vf "fps=10,scale=720:-1:flags=lanczos,split[a][b];\\
        [a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=5" demo.gif

Requires a sourced ROS 2 workspace (rosbag2_py, the message packages) plus OpenCV and numpy.
"""
import argparse
import importlib.util
import math
import os
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, os.pardir)

# ---------------------------------------------------------------------------
# Layout and palette
# ---------------------------------------------------------------------------
W, H = 1280, 768
CAM_X, CAM_Y, CAM_W, CAM_H = 16, 52, 960, 540       # camera panel
MAP_X, MAP_Y, MAP_W, MAP_H = 1000, 52, 264, 656     # tank map column

BG = (24, 22, 20)
PANEL = (38, 35, 32)
TEXT = (232, 230, 228)
DIM = (140, 136, 132)
ACCENT = (86, 180, 233)
PALETTE = {0: (114, 178, 0), 1: (0, 158, 255), 2: (196, 121, 0), 3: (66, 66, 240),
           4: (163, 108, 204), 5: (0, 200, 220), 6: (128, 190, 0)}
FONT = cv2.FONT_HERSHEY_DUPLEX


def col(tag_id):
    return PALETTE.get(tag_id, (200, 200, 200))


def txt(img, s, org, scale=0.5, color=TEXT, thick=1):
    cv2.putText(img, s, org, FONT, scale, color, thick, cv2.LINE_AA)


# ---------------------------------------------------------------------------
# The node's own maths
# ---------------------------------------------------------------------------
def load_localizer_node():
    """Import localizer_node, preferring the installed package over the source tree."""
    try:
        from apriltag_localization import localizer_node
        return localizer_node
    except ImportError:
        path = os.path.join(PKG, 'apriltag_localization', 'localizer_node.py')
        spec = importlib.util.spec_from_file_location('localizer_node', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


ln = load_localizer_node()


def load_calibration(path):
    d = yaml.safe_load(open(path))
    K = np.array(d['camera_matrix']['data'], np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], np.float64)
    return K, D


def load_tag_map(path):
    d = yaml.safe_load(open(path))
    default = d.get('default_tag_size', 0.135)
    return {int(i): (ln.tag_world_T(e['x'], e['y'], e['z'], e['theta_deg']),
                     e.get('size', default))
            for i, e in d['tags'].items()}


def tag_pose_in_camera(corners, size, K, D):
    """`VesselLocalizer.tag_pose_in_camera`: message corners are BL,BR,TR,TL and must be
    reversed to the TL,TR,BR,BL that IPPE_SQUARE expects."""
    obj = ln.object_points(size)
    img = np.array([[corners[3].x, corners[3].y], [corners[2].x, corners[2].y],
                    [corners[1].x, corners[1].y], [corners[0].x, corners[0].y]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return ln.make_T(R, tvec.flatten())


def solve_frame(dets, tag_map, K, D, T_B_C):
    """`VesselLocalizer.on_tags`: per-tag chain, then the node's own fusion.

    Fusion is delegated to VesselLocalizer.fuse() rather than reimplemented here.
    It used to be inlined, and that inlining silently went stale on 2026-08-11 when
    fusion became TWO-PASS (Open Item #9) -- this renderer would have kept drawing
    the old one-pass result while claiming to show shipped code. fuse() only touches
    self.T_B_C, so a SimpleNamespace is a sufficient stand-in for the node.
    """
    ests, ids, ranges = [], [], {}
    for tag_id, corners, margin in dets:
        if tag_id not in tag_map or margin < 30.0:
            continue
        T_C_T = tag_pose_in_camera(corners, tag_map[tag_id][1], K, D)
        if T_C_T is None:
            continue
        T_W_T = tag_map[tag_id][0]
        ranges[tag_id] = float(np.linalg.norm(T_C_T[:3, 3]))
        ests.append(ln.TagEstimate(
            tag_id=tag_id,
            T_W_B=T_W_T @ ln.invert_T(T_C_T) @ ln.invert_T(T_B_C),
            weight=margin,
            ratio=float('nan'),          # Open Item #8 diagnostic, unused by fuse()
            t_C_T=T_C_T[:3, 3],
            p_W_T=T_W_T[:3, 3]))
        ids.append(tag_id)
    if not ests:
        return None
    t, q = ln.VesselLocalizer.fuse(SimpleNamespace(T_B_C=T_B_C), ests)
    _, _, yaw = ln.quat_to_euler(q)
    return dict(ids=ids, t=t, yaw=yaw, ranges=ranges)


def camera_mount():
    """The mount currently compiled into the node, read back through its own constructor."""
    return ln.VesselLocalizer.make_camera_mount(None)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def world_to_map(x, y, tank_l, tank_w):
    """World X (down the tank) -> screen down; world Y (across) -> screen right."""
    s = min((MAP_W - 46) / tank_w, (MAP_H - 130) / tank_l)
    ox = MAP_X + (MAP_W - tank_w * s) / 2
    oy = MAP_Y + 60
    return int(ox + y * s), int(oy + x * s)


def draw_map(canvas, tag_map, recent, cloud, pose, live_ids, tank_l, tank_w):
    x0, y0 = world_to_map(0, 0, tank_l, tank_w)
    x1, y1 = world_to_map(tank_l, tank_w, tank_l, tank_w)
    cv2.rectangle(canvas, (MAP_X, MAP_Y), (MAP_X + MAP_W, MAP_Y + MAP_H), PANEL, -1)
    txt(canvas, 'TANK  (top view)', (MAP_X + 14, MAP_Y + 24), 0.48, TEXT)
    txt(canvas, 'X down / Y right, origin top-left', (MAP_X + 14, MAP_Y + 42), 0.36, DIM)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (70, 66, 62), -1)
    cv2.rectangle(canvas, (x0, y0), (x1, y1), (110, 106, 100), 1)
    for m in range(0, int(tank_l) + 1, 2):
        _, gy = world_to_map(m, 0, tank_l, tank_w)
        cv2.line(canvas, (x0, gy), (x1, gy), (58, 55, 52), 1)
        txt(canvas, f'{m}', (x0 - 18, gy + 4), 0.32, DIM)

    for tag_id, (T, _) in sorted(tag_map.items()):
        px, py = world_to_map(T[0, 3], T[1, 3], tank_l, tank_w)
        live = tag_id in live_ids
        c = col(tag_id) if live else (100, 97, 94)
        cv2.rectangle(canvas, (px - 4, py - 4), (px + 4, py + 4), c, -1 if live else 1)
        if live:
            cv2.circle(canvas, (px, py), 9, c, 1, cv2.LINE_AA)
        txt(canvas, str(tag_id),
            (px + 9 if T[1, 3] < tank_w / 2 else px - 20, py + 4), 0.36, c if live else DIM)

    # Every fix so far, as a scatter -- deliberately NOT joined by a line: consecutive fixes
    # differ by decimetres, and a polyline reads as a broken track rather than as the spread
    # it actually is.
    for cx, cy in cloud:
        px, py = world_to_map(cx, cy, tank_l, tank_w)
        cv2.circle(canvas, (px, py), 1, (96, 92, 88), -1)
    for cx, cy in recent:
        px, py = world_to_map(cx, cy, tank_l, tank_w)
        cv2.circle(canvas, (px, py), 2, (70, 128, 160), -1, cv2.LINE_AA)

    if pose is not None:
        px, py = world_to_map(pose[0], pose[1], tank_l, tank_w)
        yaw = pose[2]
        # world heading (cos yaw along X, sin yaw along Y) -> screen (right, down)
        hx, hy = math.sin(yaw), math.cos(yaw)
        tri = np.array([(int(px + hx * 19), int(py + hy * 19)),
                        (int(px - hy * 10 - hx * 7), int(py + hx * 10 - hy * 7)),
                        (int(px + hy * 10 - hx * 7), int(py - hx * 10 - hy * 7))])
        cv2.fillPoly(canvas, [tri], ACCENT, cv2.LINE_AA)
        cv2.polylines(canvas, [tri], True, (255, 255, 255), 1, cv2.LINE_AA)


def draw_insets(panel, img_full, dets, img_w):
    """Zoomed crops of each detected tag. Tags are ~15-55 px in a 1080p tank frame -- without
    this they are invisible in the rendered video."""
    n = min(len(dets), 4)
    if not n:
        return
    size, gap = 116, 10
    x = (CAM_W - (n * size + (n - 1) * gap)) // 2
    y = CAM_H - size - 14
    for tag_id, pts_full, rng in dets[:n]:
        c = col(tag_id)
        cx, cy = pts_full.mean(0)
        half = max(46, int(np.linalg.norm(pts_full[0] - pts_full[1]) * 1.6))
        a, b = max(0, int(cx - half)), max(0, int(cy - half))
        a2 = min(img_full.shape[1], int(cx + half))
        b2 = min(img_full.shape[0], int(cy + half))
        crop = img_full[b:b2, a:a2]
        if crop.size == 0:
            continue
        ch, cw = crop.shape[:2]
        crop = cv2.resize(crop, (size, size), interpolation=cv2.INTER_NEAREST)
        q = (pts_full - [a, b]) * [size / cw, size / ch]
        cv2.polylines(crop, [q.astype(np.int32)], True, c, 2, cv2.LINE_AA)
        panel[y:y + size, x:x + size] = crop
        cv2.rectangle(panel, (x, y), (x + size, y + size), c, 2)
        cv2.rectangle(panel, (x, y - 18), (x + size, y), c, -1)
        txt(panel, f'tag {tag_id}   {rng:.2f} m', (x + 8, y - 5), 0.4, (20, 20, 20))
        px, py = (pts_full.mean(0) * (CAM_W / img_w)).astype(int)
        cv2.line(panel, (px, py + 12), (x + size // 2, y - 20), c, 1, cv2.LINE_AA)
        x += size + gap


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------
def open_bag(uri, storage):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def read_tags(uri, storage, topic):
    """First pass: {stamp_ns: [(id, corners, decision_margin), ...]}.

    Two passes are needed because detections trail their image by the detection latency, so a
    single forward pass would have to buffer frames to find the matching detections.
    """
    reader, types = open_bag(uri, storage)
    if topic not in types:
        sys.exit(f'error: {uri} has no topic {topic} (has: {sorted(types)})')
    out = {}
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, get_message(types[name]))
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        out[stamp] = [(d.id, d.corners, d.decision_margin) for d in m.detections]
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag', help='bag directory')
    p.add_argument('--calib', required=True,
                   help="camera_<res>.yaml to use INSTEAD of the bag's camera_info")
    p.add_argument('--map', default=os.path.join(PKG, 'config', 'tag_map.yaml'))
    p.add_argument('--out', default='demo.mp4')
    p.add_argument('--start', type=float, default=0.0, help='seconds from bag start')
    p.add_argument('--end', type=float, default=float('inf'))
    p.add_argument('--storage', default='mcap')
    p.add_argument('--image-topic', default='/camera/image_raw')
    p.add_argument('--tags-topic', default='/tags')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--tank', default='14.0,5.0', help='drawn tank extent "length,width" (m)')
    p.add_argument('--title', default='apriltag_localization  -  vessel_localizer')
    p.add_argument('--subtitle', default='')
    args = p.parse_args()

    tank_l, tank_w = (float(v) for v in args.tank.split(','))
    K, D = load_calibration(args.calib)
    tag_map = load_tag_map(args.map)
    T_B_C = camera_mount()
    mount = T_B_C[:3, 3]
    tags = read_tags(args.bag, args.storage, args.tags_topic)

    reader, types = open_bag(args.bag, args.storage)
    ff = subprocess.Popen(
        ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
         '-s', f'{W}x{H}', '-r', str(args.fps), '-i', '-', '-c:v', 'libx264', '-crf', '24',
         '-pix_fmt', 'yuv420p', '-movflags', '+faststart', args.out], stdin=subprocess.PIPE)

    t0 = None
    recent, cloud, last = [], [], None
    n_fix = n_frame = 0
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != args.image_topic:
            continue
        m = deserialize_message(data, get_message(types[name]))
        stamp = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        if t0 is None:
            t0 = stamp
        t = (stamp - t0) / 1e9
        if t < args.start:
            continue
        if t > args.end:
            break
        n_frame += 1

        full = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            full = cv2.cvtColor(full, cv2.COLOR_RGB2BGR)
        panel = cv2.resize(full, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)

        res = solve_frame(tags[stamp], tag_map, K, D, T_B_C) if stamp in tags else None
        dets = []
        if res:
            n_fix += 1
            for tag_id, corners, _ in tags[stamp]:
                if tag_id not in res['ids']:
                    continue
                pts = np.array([[c.x, c.y] for c in corners], np.float64)
                dets.append((tag_id, pts, res['ranges'][tag_id]))
                q = (pts * (CAM_W / m.width)).astype(np.int32)
                cv2.polylines(panel, [q], True, col(tag_id), 2, cv2.LINE_AA)
                cv2.circle(panel, tuple(q.mean(0).astype(int)), 16, col(tag_id), 1, cv2.LINE_AA)
            last = (res['t'][0], res['t'][1], res['yaw'])
            cloud.append(last[:2])
            recent = (recent + [last[:2]])[-40:]

        draw_insets(panel, full, sorted(dets, key=lambda d: d[1][:, 0].mean()), m.width)

        canvas = np.full((H, W, 3), BG, np.uint8)
        canvas[CAM_Y:CAM_Y + CAM_H, CAM_X:CAM_X + CAM_W] = panel
        cv2.rectangle(canvas, (CAM_X, CAM_Y), (CAM_X + CAM_W, CAM_Y + CAM_H), (70, 66, 62), 1)
        txt(canvas, args.title, (16, 32), 0.62, TEXT)
        if args.subtitle:
            txt(canvas, args.subtitle, (470, 32), 0.42, DIM)

        hy = CAM_Y + CAM_H + 30
        txt(canvas, f't = {t:5.2f} s', (CAM_X, hy), 0.5, DIM)
        if res:
            label = f'FIX  -  {len(res["ids"])} tag' + ('s fused' if len(res['ids']) > 1 else '')
            cv2.circle(canvas, (CAM_X + 130, hy - 5), 5, (110, 220, 110), -1)
            txt(canvas, label, (CAM_X + 145, hy), 0.5, TEXT)
            txt(canvas, 'id ' + ','.join(str(i) for i in sorted(res['ids'])),
                (CAM_X + 330, hy), 0.5, DIM)
            txt(canvas, f'x {last[0]:+6.2f} m    y {last[1]:+6.2f} m    '
                        f'yaw {math.degrees(last[2]):+7.1f} deg', (CAM_X, hy + 28), 0.56, ACCENT)
        else:
            cv2.circle(canvas, (CAM_X + 130, hy - 5), 5, (70, 70, 90), -1)
            txt(canvas, 'no tag decoded this frame', (CAM_X + 145, hy), 0.5, DIM)
            if last:
                txt(canvas, f'last fix  x {last[0]:+6.2f} m   y {last[1]:+6.2f} m   '
                            f'yaw {math.degrees(last[2]):+7.1f} deg',
                    (CAM_X, hy + 28), 0.56, DIM)
        txt(canvas, f'frames with a fix, this excerpt: {100 * n_fix / max(1, n_frame):4.1f} %   '
                    '(raw per-frame output, no filtering)', (CAM_X, hy + 54), 0.42, DIM)
        txt(canvas, f'intrinsics: {os.path.basename(args.calib)}   |   camera mount t_B_C: '
                    f'[{mount[0]:.2f}, {mount[1]:.2f}, {mount[2]:.2f}] m',
            (CAM_X, hy + 76), 0.42, (110, 106, 102))

        draw_map(canvas, tag_map, recent, cloud, last,
                 set(res['ids']) if res else set(), tank_l, tank_w)
        txt(canvas, 'arrow = fused /vessel_pose', (MAP_X + 14, MAP_Y + MAP_H - 58), 0.36, DIM)
        txt(canvas, 'dots  = every fix so far', (MAP_X + 14, MAP_Y + MAP_H - 39), 0.36, DIM)
        txt(canvas, '(spread = per-frame scatter)', (MAP_X + 14, MAP_Y + MAP_H - 20), 0.34, DIM)

        ff.stdin.write(canvas.tobytes())

    ff.stdin.close()
    ff.wait()
    print(f'{n_frame} frames, {n_fix} with a fix '
          f'({100 * n_fix / max(1, n_frame):.1f}%) -> {args.out}')


if __name__ == '__main__':
    main()
