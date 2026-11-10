"""Livox SDK2 Ethernet data packets (point cloud and IMU), as pushed by a Mid-360 over UDP.

Layout follows LivoxLidarEthernetPacket in Livox-SDK2 include/livox_lidar_def.h (packed, little-endian):
  version u8 | length u16 | time_interval u16 (0.1 us) | dot_num u16 | udp_cnt u16 | frame_cnt u8 |
  data_type u8 | time_type u8 | rsvd[12] | crc32 u32 | timestamp u64 (ns) | data...
data_type 0x01 = Cartesian high precision: x, y, z int32 millimetres, reflectivity u8, tag u8 (14 bytes/point).
data_type 0x00 = IMU: gyro xyz float32 (rad/s), acc xyz float32 (g).
"""
import struct
import numpy as np

HEADER = struct.Struct('<BHHHHBBB12sIQ')  # 36 bytes
assert HEADER.size == 36
TYPE_IMU, TYPE_CART_HIGH = 0x00, 0x01
POINT_DTYPE = np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'), ('refl', 'u1'), ('tag', 'u1')])
assert POINT_DTYPE.itemsize == 14
IMU = struct.Struct('<6f')
POINTS_PER_PACKET = 96
# Default ports from Livox-SDK2 samples/livox_lidar_quick_start/mid360_config.json (host side)
HOST_POINT_PORT, HOST_IMU_PORT = 56301, 56401


def encode_points(xyz_m, refl, t0_ns, span_s, udp_cnt, frame_cnt=0):
    """One point packet. xyz_m: (n,3) metres in the sensor frame; span_s: time covered by the packet."""
    n = len(xyz_m)
    body = np.zeros(n, POINT_DTYPE)
    mm = np.round(np.asarray(xyz_m) * 1000).astype('<i4')
    body['x'], body['y'], body['z'] = mm[:, 0], mm[:, 1], mm[:, 2]
    body['refl'] = refl
    head = HEADER.pack(0, HEADER.size + n * 14, min(65535, int(round(span_s * 1e7))), n, udp_cnt & 0xFFFF,
                       frame_cnt & 0xFF, TYPE_CART_HIGH, 0, b'\0' * 12, 0, int(t0_ns))
    return head + body.tobytes()


def encode_imu(gyro, acc_g, t_ns, udp_cnt):
    head = HEADER.pack(0, HEADER.size + IMU.size, 0, 1, udp_cnt & 0xFFFF, 0, TYPE_IMU, 0, b'\0' * 12, 0, int(t_ns))
    return head + IMU.pack(*gyro, *acc_g)


def decode(buf):
    """Returns a dict. Points: {'type','t_ns','udp_cnt','xyz' (n,3) float32 m,'refl','tag','t' (n,) float64 s}.
    IMU: {'type','t_ns','udp_cnt','gyro','acc_g'}. Returns None for packets of other types."""
    if len(buf) < HEADER.size:
        return None
    _v, _length, interval, dot_num, udp_cnt, _fc, dtype, _tt, _r, _crc, t_ns = HEADER.unpack_from(buf)
    if dtype == TYPE_CART_HIGH:
        pts = np.frombuffer(buf, POINT_DTYPE, count=dot_num, offset=HEADER.size)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], 1).astype(np.float32) / 1000.0
        span = interval * 1e-7
        t = t_ns * 1e-9 + (np.arange(dot_num) / max(dot_num, 1)) * span
        return {'type': 'points', 't_ns': t_ns, 'udp_cnt': udp_cnt, 'xyz': xyz, 'refl': pts['refl'], 'tag': pts['tag'], 't': t}
    if dtype == TYPE_IMU:
        v = IMU.unpack_from(buf, HEADER.size)
        return {'type': 'imu', 't_ns': t_ns, 'udp_cnt': udp_cnt, 'gyro': v[:3], 'acc_g': v[3:]}
    return None
