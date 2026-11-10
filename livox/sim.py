"""Simulated Livox Mid-360 carried through a virtual two-room building.

Produces the same UDP payloads the real sensor pushes (see packets.py), plus ground truth, so the
recorder and mapper run unchanged on simulated and real data.

Sensor model (from the Mid-360 spec sheet): 200,000 points/s, 360 deg x (-7..52 deg) field of view,
0.1-40 m range, range noise 2 cm (1 sigma), angular noise 0.15 deg, IMU at 200 Hz.
The real unit's non-repetitive scan pattern is proprietary; here azimuth sweeps at 10 rev/s while
elevation follows a low-discrepancy sequence, which gives similar coverage per 100 ms frame.
"""
import numpy as np
from . import packets

POINT_RATE = 200_000
IMU_RATE = 200
FOV_EL = np.radians([-7.0, 52.0])
RANGE = (0.1, 40.0)
SIGMA_RANGE = 0.02
SIGMA_ANGLE = np.radians(0.15)
G = 9.81


def building():
    """Axis-aligned boxes (min, max) in metres. Two rooms joined by a doorway, with furniture."""
    W, D, H, T = 12.0, 8.0, 2.7, 0.15
    b = [
        ((0, 0, -T), (W, D, 0)), ((0, 0, H), (W, D, H + T)),            # floor, ceiling
        ((-T, 0, 0), (0, D, H)), ((W, 0, 0), (W + T, D, H)),            # west, east walls
        ((0, -T, 0), (W, 0, H)), ((0, D, 0), (W, D + T, H)),            # south, north walls
        ((6, 0, 0), (6 + T, 3.0, H)), ((6, 4.2, 0), (6 + T, D, H)),     # dividing wall, doorway y 3.0-4.2
        ((6, 3.0, 2.1), (6 + T, 4.2, H)),                               # lintel over the door
        ((1.0, 5.5, 0), (2.8, 6.5, 0.75)),                              # table
        ((0.1, 1.0, 0), (0.6, 3.0, 1.9)),                               # wardrobe
        ((3.5, 0.2, 0), (5.5, 1.1, 0.8)),                               # sofa
        ((4.3, 6.9, 0), (4.7, 7.3, H)),                                 # pillar
        ((8.0, 0.2, 0), (11.0, 0.7, 2.0)),                              # shelving
        ((9.0, 5.0, 0), (10.5, 6.5, 0.45)),                             # low platform
        ((11.3, 2.5, 0), (11.9, 4.5, 1.2)),                             # cabinet
        ((7.2, 7.2, 0), (7.9, 7.9, 1.0)),                               # crate
    ]
    return np.array(b, dtype=np.float64)


def raycast(origins, dirs, boxes):
    """Nearest hit distance for each ray against all boxes (slab method). inf where nothing is hit."""
    with np.errstate(divide='ignore', invalid='ignore'):
        inv = 1.0 / dirs
        t1 = (boxes[None, :, 0, :] - origins[:, None, :]) * inv[:, None, :]
        t2 = (boxes[None, :, 1, :] - origins[:, None, :]) * inv[:, None, :]
    tn = np.nanmax(np.minimum(t1, t2), axis=2)
    tf = np.nanmin(np.maximum(t1, t2), axis=2)
    hit = (tf >= tn) & (tf > 1e-6)
    t = np.where(tn > 1e-6, tn, tf)      # origin outside the box: entry point; inside: exit point
    t = np.where(hit, t, np.inf)
    return t.min(axis=1)


def surface_distance(points, boxes, chunk=200_000):
    """Distance from each point to the nearest box surface (ground-truth map error)."""
    out = np.empty(len(points))
    for i in range(0, len(points), chunk):
        p = points[i:i + chunk, None, :]
        lo, hi = boxes[None, :, 0, :], boxes[None, :, 1, :]
        q = np.maximum(lo - p, p - hi)                      # >0 outside along that axis
        outside = np.linalg.norm(np.maximum(q, 0), axis=2)
        inside = np.minimum(q.max(axis=2), 0)               # negative depth when inside
        out[i:i + chunk] = np.abs(outside + inside).min(axis=1)
    return out


class Trajectory:
    """Hand-carried walk: through room A, the doorway, around room B and back to the start."""

    def __init__(self, speed=0.7, height=1.3, dt=0.005, seed=1, hold=1.5):
        way = np.array([[2.0, 2.0], [4.5, 2.5], [5.0, 3.6], [7.2, 3.6], [9.5, 2.5], [10.3, 4.0], [9.0, 6.9],
                        [7.3, 5.5], [7.0, 3.6], [5.0, 3.6], [3.6, 5.2], [2.0, 4.6], [1.6, 3.0], [2.0, 2.0]])
        seg = np.linalg.norm(np.diff(way, axis=0), axis=1)
        s = np.concatenate([[0], np.cumsum(seg)])
        # stand still for `hold` seconds at the start and end, as you would with the real sensor,
        # so the first frame (which defines the map's origin) is not smeared by motion
        self.duration = float(s[-1] / speed) + 2 * hold
        t = np.arange(0, self.duration + dt, dt)
        dist = np.clip(t - hold, 0, s[-1] / speed) * speed
        xy = np.stack([np.interp(dist, s, way[:, 0]), np.interp(dist, s, way[:, 1])], 1)
        k = int(1.2 / dt) | 1                                # smooth the corners (about 1.2 s window)
        ker = np.hanning(k); ker /= ker.sum()
        pad = np.pad(xy, ((k // 2, k // 2), (0, 0)), mode='edge')
        xy = np.stack([np.convolve(pad[:, i], ker, mode='valid') for i in range(2)], 1)
        v = np.gradient(xy, dt, axis=0)
        moving = np.linalg.norm(v, axis=1) > 0.05
        head = np.arctan2(v[:, 1], v[:, 0])
        first, last = np.argmax(moving), len(moving) - 1 - np.argmax(moving[::-1])
        head[:first] = head[first]; head[last + 1:] = head[last]      # keep facing the same way while standing
        yaw = np.unwrap(head)
        pad = np.pad(yaw, (k // 2, k // 2), mode='edge')
        yaw = np.convolve(pad, ker, mode='valid')
        rng = np.random.default_rng(seed)
        ph = rng.uniform(0, 2 * np.pi, 3)
        self.t = t
        self.pos = np.column_stack([xy, height + 0.02 * np.sin(2 * np.pi * 1.8 * t + ph[0])])   # walking bob
        self.rpy = np.column_stack([np.radians(3) * np.sin(2 * np.pi * 0.9 * t + ph[1]),
                                    np.radians(3) * np.sin(2 * np.pi * 0.7 * t + ph[2]), yaw])
        self.acc = np.gradient(np.gradient(self.pos, dt, axis=0), dt, axis=0)                   # world frame

    def at(self, t):
        pos = np.stack([np.interp(t, self.t, self.pos[:, i]) for i in range(3)], -1)
        rpy = np.stack([np.interp(t, self.t, self.rpy[:, i]) for i in range(3)], -1)
        return pos, rpy


def rot(rpy):
    """Body->world rotation matrices (...,3,3) from roll, pitch, yaw (ZYX)."""
    r, p, y = rpy[..., 0], rpy[..., 1], rpy[..., 2]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty(rpy.shape[:-1] + (3, 3))
    R[..., 0, 0] = cy * cp; R[..., 0, 1] = cy * sp * sr - sy * cr; R[..., 0, 2] = cy * sp * cr + sy * sr
    R[..., 1, 0] = sy * cp; R[..., 1, 1] = sy * sp * sr + cy * cr; R[..., 1, 2] = sy * sp * cr - cy * sr
    R[..., 2, 0] = -sp;     R[..., 2, 1] = cp * sr;                R[..., 2, 2] = cp * cr
    return R


def pose_matrix(pos, rpy):
    T = np.eye(4); T[:3, :3] = rot(np.asarray(rpy)); T[:3, 3] = pos
    return T


class Mid360Sim:
    def __init__(self, traj=None, boxes=None, seed=0, noise=True, t_start_ns=1_700_000_000_000_000_000):
        self.traj = traj or Trajectory()
        self.boxes = building() if boxes is None else boxes
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.t0_ns = t_start_ns

    def scan(self, t):
        """Sensor-frame points for firing times t (seconds from start). Returns (xyz, valid mask)."""
        n = len(t)
        idx = np.round(t * POINT_RATE).astype(np.int64)
        az = 2 * np.pi * ((idx * (10.0 / POINT_RATE)) % 1.0) + 2 * np.pi * ((idx * 0.754877666) % 1.0) / 500.0
        el = FOV_EL[0] + (FOV_EL[1] - FOV_EL[0]) * ((idx * 0.6180339887498949) % 1.0)
        if self.noise:
            az = az + self.rng.normal(0, SIGMA_ANGLE, n); el = el + self.rng.normal(0, SIGMA_ANGLE, n)
        d_body = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], 1)
        pos, rpy = self.traj.at(t)
        R = rot(rpy)
        d_world = np.einsum('nij,nj->ni', R, d_body)
        r = raycast(pos, d_world, self.boxes)
        valid = np.isfinite(r) & (r >= RANGE[0]) & (r <= RANGE[1])
        if self.noise:
            r = r + self.rng.normal(0, SIGMA_RANGE, n)
        # what the sensor reports: range along the direction it believes it fired (noise-free angles are unknown to it)
        return d_body * r[:, None], valid

    def packets(self, duration=None):
        """Yields (kind, t_seconds, payload bytes) in time order: 'points' and 'imu' packets."""
        duration = min(duration or self.traj.duration, self.traj.duration)
        n_pkts = int(duration * POINT_RATE / packets.POINTS_PER_PACKET)
        pkt_span = packets.POINTS_PER_PACKET / POINT_RATE
        imu_dt, next_imu, imu_cnt = 1.0 / IMU_RATE, 0.0, 0
        batch = 256                                           # packets ray-cast together
        for b0 in range(0, n_pkts, batch):
            k = np.arange(b0, min(b0 + batch, n_pkts))
            t = (k[:, None] * packets.POINTS_PER_PACKET + np.arange(packets.POINTS_PER_PACKET)[None, :]) / POINT_RATE
            xyz, valid = self.scan(t.ravel())
            xyz = xyz.reshape(len(k), -1, 3); valid = valid.reshape(len(k), -1)
            for j, kk in enumerate(k):
                t_pkt = kk * pkt_span
                while next_imu <= t_pkt:
                    yield 'imu', next_imu, self._imu_packet(next_imu, imu_cnt)
                    imu_cnt += 1; next_imu += imu_dt
                # invalid returns are sent as (0,0,0), as the real sensor does for no-return points
                pts = np.where(valid[j][:, None], xyz[j], 0.0)
                refl = np.where(valid[j], 120, 0).astype(np.uint8)
                yield 'points', t_pkt, packets.encode_points(pts, refl, self.t0_ns + int(round(t_pkt * 1e9)), pkt_span, kk)

    def _imu_packet(self, t, cnt, h=5e-3):
        _pos, rpy = self.traj.at(np.array([t - h, t, t + h]))
        R = rot(rpy)
        acc_w = np.array([np.interp(t, self.traj.t, self.traj.acc[:, i]) for i in range(3)]) + np.array([0, 0, G])  # specific force
        dR = R[0].T @ R[2]                                                       # body rotation over 2h
        gyro = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (4 * h)
        acc_g = (R[1].T @ acc_w) / G
        if self.noise:
            gyro = gyro + self.rng.normal(0, 0.002, 3); acc_g = acc_g + self.rng.normal(0, 0.002, 3)
        return packets.encode_imu(gyro, acc_g, self.t0_ns + int(round(t * 1e9)), cnt)

    def true_pose(self, t):
        pos, rpy = self.traj.at(np.array([t]))
        return pose_matrix(pos[0], rpy[0])
