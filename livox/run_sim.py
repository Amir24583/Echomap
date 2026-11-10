"""Simulate a Mid-360 walk through the virtual building, map it, and score the result against ground truth.

    python -m livox.run_sim --out out/            # whole walk, packets written straight to the log
    python -m livox.run_sim --udp --seconds 5     # also push packets through real UDP sockets into the recorder
"""
import argparse, json, os, socket, threading, time
import numpy as np
from . import packets, sim as S
from .mapper import build_map, write_ply
from .record import LogWriter, record


def simulate_to_log(path, sim, seconds=None):
    w = LogWriter(path)
    for kind, t, payload in sim.packets(seconds):
        w.write(kind, t, payload)
    w.close()
    return w.count


def simulate_over_udp(path, sim, seconds, host='127.0.0.1'):
    """Send simulated packets to the recorder through the loopback interface, paced at 4x real time."""
    ready, out = threading.Event(), {}
    th = threading.Thread(target=lambda: out.update(record(path, bind=host, idle_timeout=1.5, on_ready=ready.set)))
    th.start(); ready.wait(5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = {'points': packets.HOST_POINT_PORT, 'imu': packets.HOST_IMU_PORT}
    t_wall, sent = time.time(), 0
    for kind, t, payload in sim.packets(seconds):
        lag = t / 4.0 - (time.time() - t_wall)
        if lag > 0:
            time.sleep(lag)
        tx.sendto(payload, (host, port[kind])); sent += 1
    th.join()
    out['sent'] = sent
    return out


def rigid_fit(src, dst):
    """Least-squares rotation + translation (no scale) taking src points onto dst (Kabsch)."""
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cd - R @ cs


def evaluate(pts, poses, sim):
    """Path error: estimated vs true sensor position after the standard rigid alignment of the two paths
    (the map's own origin is arbitrary). Map error: distance of each map point from the true surfaces,
    using that same alignment."""
    est = np.array([T[:3, 3] for _t, T in poses])
    tru = np.array([sim.true_pose(min(t, sim.traj.duration))[:3, 3] for t, _T in poses])
    R, tr = rigid_fit(est, tru)
    e = np.linalg.norm(est @ R.T + tr - tru, axis=1)
    world = pts @ R.T + tr
    d = S.surface_distance(world, sim.boxes)
    # loop closure: the walk ends where it started, so the first and last estimated positions should agree
    gap = np.linalg.norm(est[-1] - est[0]) - np.linalg.norm(tru[-1] - tru[0])
    path_len = float(np.linalg.norm(np.diff(tru, axis=0), axis=1).sum())
    r = lambda v: round(float(v) * 100, 2)
    return {
        'map_points': int(len(pts)),
        'map_error_cm': {'median': r(np.median(d)), 'p90': r(np.percentile(d, 90)), 'p99': r(np.percentile(d, 99))},
        'path_error_cm': {'rmse': r(np.sqrt((e ** 2).mean())), 'max': r(e.max())},
        'start_to_end_drift_cm': r(abs(gap)),
        'path_length_m': round(path_len, 2), 'frames': len(poses), 'seconds': round(poses[-1][0], 1),
    }, world


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='out'); ap.add_argument('--seconds', type=float)
    ap.add_argument('--udp', action='store_true'); ap.add_argument('--no-noise', action='store_true')
    ap.add_argument('--voxel', type=float, default=0.03)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    sim = S.Mid360Sim(noise=not a.no_noise)
    log = os.path.join(a.out, 'sim.lvx-log')
    t0 = time.time()
    if a.udp:
        st = simulate_over_udp(log, sim, a.seconds or 5.0)
        print('udp:', st)
    else:
        n = simulate_to_log(log, sim, a.seconds)
        print(f'simulated {n:,} packets in {time.time() - t0:.1f}s')
    t0 = time.time()
    pts, poses = build_map(log, voxel=a.voxel)
    print(f'mapped {len(poses)} frames in {time.time() - t0:.1f}s')
    report, world = evaluate(pts, poses, sim)
    write_ply(os.path.join(a.out, 'sim-map.ply'), world, z_range=(0.0, 2.7))
    with open(os.path.join(a.out, 'report.json'), 'w') as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
