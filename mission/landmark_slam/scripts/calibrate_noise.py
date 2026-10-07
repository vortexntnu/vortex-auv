#!/usr/bin/env python3
"""One-time noise calibration for landmark_slam.

Usage:
    ros2 run landmark_slam calibrate_noise.py <bag> <truth.yaml> <classes.yaml>

<bag>: a pool run near landmarks whose poses are known. It needs the
odometry (nav_msgs/Odometry) and the detections (vortex_msgs/LandmarkArray);
/tf_static if the detections are in a camera frame; /tf with map -> odom
from landmark_slam for the odometry noise.
<truth.yaml>: the true landmark poses, prior_map.yaml format (initial_pose
gives where odom starts in that frame). <classes.yaml>: landmark_classes.yaml.

Prints bearing_sigma, range_sigma_a, range_sigma_b (and, with map -> odom in
the bag, odom_sigma_trans_per_m, odom_sigma_yaw_per_m) for params.yaml.
"""

import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

MAX_MATCH_M = 1.0  # detection to nearest true landmark of its class
RANGE_BIN_M = 1.0
MIN_PER_BIN = 20
KEYFRAME_DIST_M = 0.5  # odometry steps, as keyframe_dist_m
MAD = 1.4826  # robust std = MAD * median absolute deviation


def quat_to_mat(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_mat(p, q):
    m4 = np.eye(4)
    m4[:3, :3] = quat_to_mat(q)
    m4[:3, 3] = p
    return m4


def yaw_of(m4):
    return np.arctan2(m4[1, 0], m4[0, 0])


def ypr_mat(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = (
        np.cos(yaw),
        np.sin(yaw),
        np.cos(pitch),
        np.sin(pitch),
        np.cos(roll),
        np.sin(roll),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def robust_std(x):
    x = np.asarray(x)
    return MAD * np.median(np.abs(x - np.median(x)))


def read_bag(path):
    with open(f'{path}/metadata.yaml') as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    def pick(msg_type, suffix):
        names = [n for n, t in types.items() if t == msg_type]
        if len(names) > 1:
            names = [n for n in names if n.endswith(suffix)] or names
        if len(names) != 1:
            sys.exit(f'need one {msg_type} topic (ending {suffix}), found {names}')
        return names[0]

    odom_topic = pick('nav_msgs/msg/Odometry', '/odom')
    det_topic = pick('vortex_msgs/msg/LandmarkArray', '/landmarks')
    print(f'odometry: {odom_topic}, detections: {det_topic}')

    odom, dets, static, map_odom = [], [], {}, []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (odom_topic, det_topic, '/tf', '/tf_static'):
            continue
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == odom_topic:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom.append((t, pose_mat([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]), msg))
        elif topic == det_topic:
            dets.append(msg)
        else:
            for tf in msg.transforms:
                tr, q = tf.transform.translation, tf.transform.rotation
                m4 = pose_mat([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
                if topic == '/tf_static':
                    static[tf.child_frame_id] = (tf.header.frame_id, m4)
                elif tf.header.frame_id.endswith('map') and tf.child_frame_id.endswith(
                    'odom'
                ):
                    s = tf.header.stamp
                    map_odom.append((s.sec + s.nanosec * 1e-9, m4))
    if not odom or not dets:
        sys.exit('no odometry or no detections in the bag')
    return odom, dets, static, map_odom


def static_to(static, frame, target):
    """Static transform target <- frame from /tf_static, or None."""
    m4 = np.eye(4)
    while frame != target:
        if frame not in static:
            return None
        parent, parent_from_frame = static[frame]
        m4 = parent_from_frame @ m4
        frame = parent
    return m4


def odom_at(odom, times, t):
    """Odometry pose at t: interpolated position, nearest orientation."""
    i = int(np.clip(np.searchsorted(times, t), 1, len(times) - 1))
    t0, m0, _ = odom[i - 1]
    t1, m1, _ = odom[i]
    a = 0.0 if t1 == t0 else np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
    m4 = (m0 if a < 0.5 else m1).copy()
    m4[:3, 3] = (1 - a) * m0[:3, 3] + a * m1[:3, 3]
    return m4


def map_to_odom(truth, odom_base0):
    """As landmark_slam at startup: the start pose from initial_pose."""
    ip = truth.get('initial_pose', {})
    yaw0 = ip.get('yaw', 0.0)
    r0 = odom_base0[:3, :3]
    pitch = np.arcsin(-np.clip(r0[2, 0], -1, 1))
    roll = np.arctan2(r0[2, 1], r0[2, 2])
    x0 = np.eye(4)
    x0[:3, :3] = ypr_mat(yaw0, pitch, roll)
    x0[:3, 3] = [ip.get('x', 0.0), ip.get('y', 0.0), ip.get('z', odom_base0[2, 3])]
    return x0 @ np.linalg.inv(odom_base0)


def detection_errors(odom, dets, static, truth, classes):
    times = np.array([o[0] for o in odom])
    odom_frame = odom[0][2].header.frame_id
    base_frame = odom[0][2].child_frame_id
    odom_from_map = np.linalg.inv(map_to_odom(truth, odom[0][1]))
    by_class = {}
    for lm in truth['landmarks']:
        c = classes[lm['class']]
        key = (c['type'], c['subtype'])
        by_class.setdefault(key, []).append(
            (odom_from_map @ np.array([lm['x'], lm['y'], lm['z'], 1.0]))[:3]
        )

    bearing, ranges, range_err = [], [], []
    for msg in dets:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        odom_base = odom_at(odom, times, t)
        if msg.header.frame_id == odom_frame:
            base_from_frame = np.linalg.inv(odom_base)
        else:
            base_from_frame = static_to(static, msg.header.frame_id, base_frame)
            if base_from_frame is None:
                sys.exit(f'no /tf_static from {msg.header.frame_id} to {base_frame}')
        base_from_odom = np.linalg.inv(odom_base)
        for lm in msg.landmarks:
            candidates = by_class.get((lm.type.value, lm.subtype.value))
            if not candidates:
                continue
            p = lm.pose.pose.position
            meas = (base_from_frame @ np.array([p.x, p.y, p.z, 1.0]))[:3]
            trues = [(base_from_odom @ np.append(c, 1.0))[:3] for c in candidates]
            true = min(trues, key=lambda q: np.linalg.norm(q - meas))
            if np.linalg.norm(true - meas) > MAX_MATCH_M:
                continue
            r = np.linalg.norm(true)
            u = true / r
            e1 = np.cross(u, [0.0, 0.0, 1.0])
            if np.linalg.norm(e1) < 1e-6:
                e1 = np.cross(u, [1.0, 0.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(u, e1)
            m = meas / np.linalg.norm(meas)
            bearing += [m @ e1, m @ e2]
            ranges.append(r)
            range_err.append(np.linalg.norm(meas) - r)
    return np.array(bearing), np.array(ranges), np.array(range_err)


def fit_range(ranges, err):
    rows = []
    for lo in np.arange(0.0, ranges.max() + RANGE_BIN_M, RANGE_BIN_M):
        sel = (ranges >= lo) & (ranges < lo + RANGE_BIN_M)
        if sel.sum() >= MIN_PER_BIN:
            rows.append((ranges[sel].mean(), robust_std(err[sel]), sel.sum()))
    if len(rows) < 2:
        return max(robust_std(err), 0.01), 0.0, rows
    r, s, n = (np.array(c) for c in zip(*rows))
    w = np.sqrt(n)
    a_mat = np.stack([np.ones_like(r), r], axis=1) * w[:, None]
    a, b = np.linalg.lstsq(a_mat, s * w, rcond=None)[0]
    if b < 0.0:
        a, b = np.average(s, weights=n), 0.0
    return max(a, 0.01), b, rows


def odom_noise(odom, map_odom):
    """Odometry noise from the map -> odom corrections.

    The change of the correction over one odometry step is the odometry error
    over that step as the graph saw it.
    """
    if len(map_odom) < 2:
        return None
    tc = np.array([m[0] for m in map_odom])
    trans, yaw = [], []
    last_odom, last_corr, dist = None, None, 0.0
    for t, m4, _ in odom:
        if t < tc[0] or t > tc[-1]:
            continue
        corr = map_odom[int(np.clip(np.searchsorted(tc, t), 0, len(tc) - 1))][1]
        if last_odom is None:
            last_odom, last_corr = m4, corr
            continue
        dist += np.linalg.norm(m4[:3, 3] - last_odom[:3, 3])
        last_odom = m4
        if dist >= KEYFRAME_DIST_M:
            delta = np.linalg.inv(last_corr) @ corr
            trans += list(delta[:2, 3])
            yaw.append(yaw_of(delta))
            last_corr, dist = corr, 0.0
    if len(yaw) < 10:
        return None
    zero_mad = lambda x: MAD * np.median(np.abs(x))  # noqa: E731, bias counts
    return (
        zero_mad(trans) / KEYFRAME_DIST_M,
        zero_mad(yaw) / KEYFRAME_DIST_M,
        len(yaw),
    )


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    bag, truth_file, classes_file = sys.argv[1:]
    with open(truth_file) as f:
        truth = yaml.safe_load(f)
    with open(classes_file) as f:
        classes = yaml.safe_load(f)
    odom, dets, static, map_odom = read_bag(bag)

    bearing, ranges, range_err = detection_errors(odom, dets, static, truth, classes)
    if len(ranges) < MIN_PER_BIN:
        sys.exit(f'only {len(ranges)} detections near a true landmark')
    a, b, rows = fit_range(ranges, range_err)
    print(f'{len(ranges)} detections matched to true landmarks')
    print(f'range bias (median) {np.median(range_err):+.3f} m')
    for r, s, n in rows:
        print(f'  range {r:5.2f} m: sigma {s:.3f} m ({n} detections)')
    print()
    print('# params.yaml')
    print(f'bearing_sigma: {robust_std(bearing):.4f}')
    print(f'range_sigma_a: {a:.4f}')
    print(f'range_sigma_b: {b:.4f}')

    noise = odom_noise(odom, map_odom)
    if noise is None:
        print('# no map -> odom in the bag: odometry noise not measured')
    else:
        print(f'odom_sigma_trans_per_m: {noise[0]:.4f}')
        print(f'odom_sigma_yaw_per_m: {noise[1]:.4f}  # from {noise[2]} steps')


if __name__ == '__main__':
    main()
