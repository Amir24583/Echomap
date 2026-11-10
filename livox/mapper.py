"""Turn a recorded Livox log into a 3D map.

Steps: decode packets -> 100 ms frames with per-point times -> LiDAR odometry (KISS-ICP: motion
de-skew + point-to-point ICP against a local voxel map) -> every frame placed with its pose ->
voxel-averaged global map -> PLY.

    python -m livox.mapper run.lvx-log map.ply
"""
import argparse
import numpy as np
from kiss_icp.config import KISSConfig
from kiss_icp.kiss_icp import KissICP
from . import packets
from .record import read_log

FRAME_SEC = 0.1


def frames(log_path, min_range=0.3, max_range=40.0):
    """Yields (t_start, xyz (n,3) float64, rel_time (n,) in 0..1) per 100 ms frame. Times in sensor seconds
    relative to the first point packet."""
    t_first, cur, xs, ts = None, None, [], []
    for kind, _rx, payload in read_log(log_path):
        if kind != 'points':
            continue
        p = packets.decode(payload)
        if p is None:
            continue
        if t_first is None:
            t_first = p['t_ns'] * 1e-9
        t = p['t'] - t_first
        r = np.linalg.norm(p['xyz'], axis=1)
        keep = (r >= min_range) & (r <= max_range)          # also drops the (0,0,0) no-return points
        idx = int(t[0] / FRAME_SEC + 1e-9)
        if cur is not None and idx != cur and xs:
            yield _emit(cur, xs, ts)
            xs, ts = [], []
        cur = idx
        xs.append(p['xyz'][keep]); ts.append(t[keep])
    if xs:
        yield _emit(cur, xs, ts)


def _emit(idx, xs, ts):
    xyz = np.concatenate(xs).astype(np.float64); t = np.concatenate(ts)
    t0 = idx * FRAME_SEC
    return t0, xyz, np.clip((t - t0) / FRAME_SEC, 0.0, 1.0)


class VoxelMap:
    """Running per-voxel mean of all points that fall in each voxel."""

    def __init__(self, voxel):
        self.voxel = voxel
        self.keys = np.empty(0, np.int64); self.sum = np.empty((0, 3)); self.n = np.empty(0, np.int64)

    def add(self, pts):
        ijk = np.floor(pts / self.voxel).astype(np.int64) + (1 << 20)
        key = (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]
        uk, inv = np.unique(key, return_inverse=True)
        s = np.zeros((len(uk), 3)); np.add.at(s, inv, pts)
        n = np.bincount(inv, minlength=len(uk))
        allk = np.concatenate([self.keys, uk])
        uk2, inv2 = np.unique(allk, return_inverse=True)
        s2 = np.zeros((len(uk2), 3)); n2 = np.zeros(len(uk2), np.int64)
        np.add.at(s2, inv2, np.concatenate([self.sum, s])); np.add.at(n2, inv2, np.concatenate([self.n, n]))
        self.keys, self.sum, self.n = uk2, s2, n2

    def points(self, min_hits=1):
        m = self.n >= min_hits
        return self.sum[m] / self.n[m, None]


def build_map(log_path, voxel=0.03, icp_voxel=0.1, max_range=40.0, min_hits=2, progress=None):
    """Returns (map points (n,3), poses list of (t_end, 4x4)). Poses are sensor->map, map = first frame."""
    cfg = KISSConfig()
    cfg.data.max_range = max_range; cfg.data.min_range = 0.3; cfg.data.deskew = True
    cfg.mapping.voxel_size = icp_voxel
    odo = KissICP(cfg)
    vmap, poses, batch = VoxelMap(voxel), [], []
    for i, (t0, xyz, rel) in enumerate(frames(log_path, max_range=max_range)):
        if len(xyz) < 500:
            continue
        deskewed, _src = odo.register_frame(xyz, rel)
        T = odo.last_pose.copy()
        poses.append((t0 + FRAME_SEC, T))                   # KISS-ICP de-skews to the end of the frame
        batch.append(deskewed @ T[:3, :3].T + T[:3, 3])
        if len(batch) >= 20:
            vmap.add(np.concatenate(batch)); batch = []
        if progress and i % 50 == 0:
            progress(i, t0)
    if batch:
        vmap.add(np.concatenate(batch))
    return vmap.points(min_hits), poses


def write_ply(path, pts, z_range=None):
    """Binary PLY with a height colour ramp (blue low, green mid, yellow high)."""
    z = pts[:, 2]
    lo, hi = z_range or (np.percentile(z, 2), np.percentile(z, 98))
    t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)[:, None]
    a, b, c = np.array([26, 51, 191]), np.array([26, 204, 140]), np.array([250, 217, 64])
    rgb = np.where(t < 0.5, a + (b - a) * (t * 2), b + (c - b) * (t * 2 - 1)).astype(np.uint8)
    rec = np.empty(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    rec['x'], rec['y'], rec['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    rec['r'], rec['g'], rec['b'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, 'wb') as f:
        f.write((f'ply\nformat binary_little_endian 1.0\ncomment EchoMap Livox map, metres\nelement vertex {len(pts)}\n'
                 'property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\n'
                 'property uchar blue\nend_header\n').encode())
        f.write(rec.tobytes())


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log'); ap.add_argument('out'); ap.add_argument('--voxel', type=float, default=0.03)
    a = ap.parse_args()
    pts, poses = build_map(a.log, voxel=a.voxel, progress=lambda i, t: print(f'frame {i}  t={t:.1f}s'))
    write_ply(a.out, pts)
    print(f'{len(pts):,} map points, {len(poses)} poses -> {a.out}')
