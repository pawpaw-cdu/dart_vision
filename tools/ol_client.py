#!/usr/bin/env python3
"""定舵角开环探针：持续向仿真器下发**固定**的 DART 指令，并把 STATE 存成 CSV。

用法: ol_client.py <pitch_deg> <yaw_deg> <roll_deg> <秒数> <输出csv>

为什么需要它：要判断"视觉算出来的三轴舵角"在物理上真正产生了什么，
最直接的办法就是绕过视觉/制导律，直接给一个已知的固定舵角，然后从轨迹
反算加速度。任何"映射错了/某一片不出力"的问题都会在这里现形。
"""

from __future__ import annotations

import csv
import math
import select
import socket
import sys
import time


def main() -> int:
    if len(sys.argv) < 6:
        print(__doc__)
        return 2
    p_deg, y_deg, r_deg = (float(v) for v in sys.argv[1:4])
    secs = float(sys.argv[4])
    out_path = sys.argv[5]
    cmd = "DART {:.6f} {:.6f} {:.6f}\n".format(
        math.radians(p_deg), math.radians(y_deg), math.radians(r_deg)
    ).encode()

    sock = socket.create_connection(("127.0.0.1", 7667), timeout=5.0)
    sock.setblocking(False)
    handle = open(out_path, "w", newline="")
    writer = csv.writer(handle)
    writer.writerow(["wall_s", "sim_t_ms", "x", "y", "z", "vx", "vy", "vz",
                     "qw", "qx", "qy", "qz", "launched", "hit", "avail",
                     "recv_wall_us"])

    started = time.time()
    next_send = 0.0
    buffer = b""
    rows = 0
    try:
        while time.time() - started < secs:
            now = time.time() - started
            if now >= next_send:
                try:
                    sock.sendall(cmd)
                except (BlockingIOError, InterruptedError):
                    pass
                next_send = now + 0.02          # 50 Hz，远高于 command_timeout 0.3 s
            ready, _, _ = select.select([sock], [], [], 0.005)
            if ready:
                try:
                    chunk = sock.recv(65536)
                except (BlockingIOError, InterruptedError):
                    chunk = b""
                if chunk == b"" and ready:
                    break
                buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode(errors="replace").strip()
                if not text.startswith("STATE "):
                    continue
                parts = text.split()
                if len(parts) < 16:
                    continue
                values = parts[1:]
                try:
                    parsed = [float(v) for v in values]
                except ValueError:
                    continue
                writer.writerow([f"{time.time() - started:.4f}"] + values)
                rows += 1
    finally:
        handle.close()
        sock.close()
    print(f"ol_client: 指令 DART pitch={p_deg:+.2f} yaw={y_deg:+.2f} roll={r_deg:+.2f} deg, "
          f"收到 {rows} 条 STATE -> {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
