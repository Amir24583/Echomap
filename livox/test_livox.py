"""Checks for the Livox pipeline. Run:  python -m livox.test_livox"""
import os, tempfile
import numpy as np
from . import packets, sim as S
from .mapper import build_map
from .record import read_log
from .run_sim import evaluate, simulate_over_udp, simulate_to_log


def test_packets():
    xyz = np.array([[1.234, -5.678, 0.5], [0, 0, 0], [39.999, 0.001, -2.0]])
    buf = packets.encode_points(xyz, [10, 0, 255], 1_700_000_000_123_456_789, 96 / 200000, 65537)
    assert len(buf) == 36 + 3 * 14
    p = packets.decode(buf)
    assert p['type'] == 'points' and p['udp_cnt'] == 1 and p['t_ns'] == 1_700_000_000_123_456_789
    assert np.allclose(p['xyz'], xyz, atol=5e-4) and list(p['refl']) == [10, 0, 255]
    q = packets.decode(packets.encode_imu([0.1, -0.2, 0.3], [0, 0, 1], 42, 7))
    assert q['type'] == 'imu' and np.allclose(q['gyro'], [0.1, -0.2, 0.3]) and np.allclose(q['acc_g'], [0, 0, 1])


def test_raycast():
    boxes = S.building()
    o = np.array([[2.0, 2.0, 1.3]] * 3)
    d = np.array([[0, 0, -1.0], [0, 0, 1.0], [-1.0, 0, 0]])
    assert np.allclose(S.raycast(o, d, boxes), [1.3, 1.4, 1.4])     # floor, ceiling, wardrobe front at x=0.6
    assert np.allclose(S.surface_distance(np.array([[2.0, 2.0, 0.05], [3.0, 4.0, 2.7]]), boxes), [0.05, 0.0])


def test_imu_at_rest():
    sim = S.Mid360Sim(noise=False)
    for kind, t, payload in sim.packets(0.05):
        if kind == 'imu':
            q = packets.decode(payload)
            assert abs(np.linalg.norm(q['acc_g']) - 1.0) < 0.05, q['acc_g']   # gravity only while standing
            return
    raise AssertionError('no imu packet')


def test_map_accuracy(tmp):
    sim = S.Mid360Sim(seed=0)
    log = os.path.join(tmp, 'a.lvx-log')
    simulate_to_log(log, sim, 9.0)
    pts, poses = build_map(log)
    rep, _ = evaluate(pts, poses, sim)
    print('  9 s walk:', rep['map_error_cm'], rep['path_error_cm'])
    assert rep['frames'] >= 88
    assert rep['map_error_cm']['median'] < 3.0 and rep['map_error_cm']['p99'] < 10.0
    assert rep['path_error_cm']['rmse'] < 5.0


def test_udp_recorder(tmp):
    sim = S.Mid360Sim(seed=0)
    log = os.path.join(tmp, 'u.lvx-log')
    st = simulate_over_udp(log, sim, 1.5)
    n = sum(1 for _ in read_log(log))
    print('  udp:', st)
    assert st['dropped'] == 0 and n == st['sent'] == st['points'] + st['imu']


if __name__ == '__main__':
    with tempfile.TemporaryDirectory() as tmp:
        for name, fn in list(globals().items()):
            if name.startswith('test_'):
                print(name)
                fn(tmp) if fn.__code__.co_argcount else fn()
    print('ok')
