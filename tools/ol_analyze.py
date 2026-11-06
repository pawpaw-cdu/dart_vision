#!/usr/bin/env python3
"""从"定舵角"飞行日志反算控制力：a = g + drag/m + F_ctrl/m  →  F_ctrl。

这条路径不需要参考弹道：日志里每一步都有位置/速度，有限差分就得到合加速度，
减去重力和（已知模型的）阻力，剩下的必然是舵面气动力。
纯弹道段（四片舵角=0）的残差应当≈0 —— 那就是这个方法的自校验。
再把残差用四元数转回机体系，就看得到"指令轴 → 真实力方向"的映射矩阵。

用法: ol_analyze.py flight_log_<tag>.csv [...]
"""

from __future__ import annotations

import csv
import math
import sys

RHO = 1.225
AREA = 0.018          # [aero] reference_area
CD = 0.22             # [aero] drag_coefficient
MASS = 0.39           # [aero] mass_kg
G = 9.81


def quat_rotate_inv(q, v):
    """机体系 -> 世界系的逆：用共轭四元数把世界系向量转回机体系。"""
    w, x, y, z = q
    # 共轭
    w, x, y, z = w, -x, -y, -z
    vx, vy, vz = v
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def load(path):
    shots, cur = [], None
    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row.get("launched") == "1":
                cur = [row] if cur is None else cur + [row]
            else:
                if cur:
                    shots.append(cur)
                    cur = None
    if cur:
        shots.append(cur)
    return shots


def analyse_shot(shot):
    f = lambda r, k: float(r[k])
    out = []
    for a, b in zip(shot, shot[1:]):
        dt = f(b, "t") - f(a, "t")
        if dt <= 1e-4:
            continue
        vx0, vy0, vz0 = f(a, "vx"), f(a, "vy"), f(a, "vz")
        vx1, vy1, vz1 = f(b, "vx"), f(b, "vy"), f(b, "vz")
        ax = (vx1 - vx0) / dt
        ay = (vy1 - vy0) / dt
        az = (vz1 - vz0) / dt
        # 用中点速度算阻力（阻力在区间内变化不大）
        mx, my, mz = (vx0 + vx1) / 2, (vy0 + vy1) / 2, (vz0 + vz1) / 2
        speed = math.sqrt(mx * mx + my * my + mz * mz)
        if speed < 1e-3:
            continue
        q = 0.5 * RHO * speed * speed * AREA
        drag = q * CD
        fx = MASS * ax - (-drag * mx / speed)
        fy = MASS * (ay + G) - (-drag * my / speed)
        fz = MASS * az - (-drag * mz / speed)
        quat = (f(b, "qw"), f(b, "qx"), f(b, "qy"), f(b, "qz"))
        body = quat_rotate_inv(quat, (fx, fy, fz))
        fins = [f(b, f"fin{k}") for k in range(1, 5)]
        spd_body = speed
        out.append(dict(
            t=f(b, "t"), z=f(b, "z"), y=f(b, "y"), x=f(b, "x"),
            speed=spd_body, force_world=(fx, fy, fz), force_body=body,
            fins=fins,
        ))
    return out


def main():
    for path in sys.argv[1:]:
        shots = load(path)
        print(f"\n===== {path}  共 {len(shots)} 发 =====")
        for i, shot in enumerate(shots):
            samples = analyse_shot(shot)
            if not samples:
                continue
            # 飞行前段（舵面还没动）与中段分开看：前段是自校验
            early = [s for s in samples if abs(s["z"]) > 15.0]
            mid = [s for s in samples if 4.0 < s["z"] < 22.0]

            def mean_body(rows):
                n = len(rows)
                if not n:
                    return (0.0, 0.0, 0.0), 0.0
                bx = sum(r["force_body"][0] for r in rows) / n
                by = sum(r["force_body"][1] for r in rows) / n
                bz = sum(r["force_body"][2] for r in rows) / n
                sp = sum(r["speed"] for r in rows) / n
                return (bx, by, bz), sp

            (ex, ey, ez), esp = mean_body(early)
            (mx_, my_, mz_), msp = mean_body(mid)
            fin_mean = [sum(abs(s["fins"][k]) for s in mid) / max(1, len(mid)) for k in range(4)]
            fin_sign = [sum(s["fins"][k] for s in mid) / max(1, len(mid)) for k in range(4)]
            print(f"  第{i+1}发 早期(自校验, |z|>15m, v={esp:5.1f}) F_body=({ex:+.3f},{ey:+.3f},{ez:+.3f}) N")
            print(f"        中段(4<z<22m,   v={msp:5.1f}) F_body=({mx_:+.3f},{my_:+.3f},{mz_:+.3f}) N"
                  f"  等效加速度=({mx_/MASS:+.2f},{my_/MASS:+.2f},{mz_/MASS:+.2f}) m/s²")
            print(f"        舵角均值(带符号) " + " ".join(f"f{k+1}={fin_sign[k]:+.3f}" for k in range(4))
                  + "   |θ|均值 " + " ".join(f"{v:.3f}" for v in fin_mean))


if __name__ == "__main__":
    main()
