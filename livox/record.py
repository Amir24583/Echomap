"""Record Livox UDP data packets to a log file, and read them back.

Log format (.lvx-log): repeated records of  kind u8 (0 imu, 1 points) | host receive time f64 | length u16 | payload.
The payload is the untouched UDP datagram, so nothing is lost for later reprocessing.

Real sensor: the Mid-360 pushes point data to the host's port 56301 and IMU data to 56401 once it is
configured with the host's IP (Livox Viewer 2 or the SDK2 config file). Run:
    python -m livox.record out.lvx-log --bind 0.0.0.0
"""
import argparse, select, socket, struct, time
from . import packets

REC = struct.Struct('<BdH')
KIND = {'imu': 0, 'points': 1}
KIND_NAME = {v: k for k, v in KIND.items()}


class LogWriter:
    def __init__(self, path):
        self.f = open(path, 'wb')
        self.count = 0

    def write(self, kind, t, payload):
        self.f.write(REC.pack(KIND[kind], t, len(payload))); self.f.write(payload); self.count += 1

    def close(self):
        self.f.close()


def read_log(path):
    """Yields (kind, receive_time, payload)."""
    with open(path, 'rb') as f:
        while True:
            h = f.read(REC.size)
            if len(h) < REC.size:
                return
            kind, t, n = REC.unpack(h)
            yield KIND_NAME[kind], t, f.read(n)


def record(path, bind='0.0.0.0', point_port=packets.HOST_POINT_PORT, imu_port=packets.HOST_IMU_PORT,
           duration=None, idle_timeout=None, on_ready=None):
    """Listen for point and IMU packets and write them to path. Returns stats, including dropped packets
    (gaps in the point packets' udp_cnt)."""
    socks = {}
    for kind, port in (('points', point_port), ('imu', imu_port)):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
        s.bind((bind, port)); s.setblocking(False)
        socks[s] = kind
    w = LogWriter(path)
    stats = {'points': 0, 'imu': 0, 'dropped': 0}
    last_cnt, t0, last_rx = None, time.time(), None
    if on_ready:
        on_ready()
    try:
        while True:
            now = time.time()
            if duration and now - t0 > duration:
                break
            if idle_timeout and last_rx and now - last_rx > idle_timeout:
                break
            ready, _, _ = select.select(list(socks), [], [], 0.2)
            for s in ready:
                while True:
                    try:
                        buf = s.recv(2048)
                    except BlockingIOError:
                        break
                    kind = socks[s]
                    last_rx = time.time()
                    w.write(kind, last_rx, buf)
                    stats[kind] += 1
                    if kind == 'points' and len(buf) >= packets.HEADER.size:
                        cnt = packets.HEADER.unpack_from(buf)[4]
                        if last_cnt is not None:
                            stats['dropped'] += (cnt - last_cnt - 1) & 0xFFFF
                        last_cnt = cnt
    except KeyboardInterrupt:
        pass
    finally:
        w.close()
        for s in socks:
            s.close()
    return stats


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out'); ap.add_argument('--bind', default='0.0.0.0'); ap.add_argument('--seconds', type=float)
    a = ap.parse_args()
    print('recording, Ctrl-C to stop')
    print(record(a.out, a.bind, duration=a.seconds))
