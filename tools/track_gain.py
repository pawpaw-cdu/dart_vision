#!/usr/bin/env python3
"""横向/竖向"跟踪增益"：镖过靶面时的实际位置 对 靶位 做线性回归。

这是判断"yaw 制导跟不跟得上"的唯一干净指标：
  斜率 0   = 制导完全没作用（镖永远飞向基地中心）
  斜率 1   = 完全跟踪（镖的横向位置跟着靶走）
用法: track_gain.py flight_log_<tag>.csv [...]
"""

from __future__ import annotations

import csv
import math
import sys

MASS = 0.39


def shots(path):
    out, cur = [], None
    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row.get("launched") == "1":
                cur = [row] if cur is None else cur + [row]
            else:
                if cur:
                    out.append(cur)
                    cur = None
    if cur:
        out.append(cur)


    return out


def plane_crossing(shot, target_z):
    ref = None
    for a, b in zip(shot, shot[1:]):
        za, zb = float(a["z"]), float(b["z"])
        if za < target_z <= zb and abs(zb - za) > 1e-9:
            ref = (a, b, (target_z - za) / (zb - za))
    if ref is None:
        return None
    a, b, frac = ref

    def lin(key):
        return float(a[key]) + (float(b[key]) - float(a[key])) * frac

    vx, vy, vz = lin("vx"), lin("vy"), lin("vz")
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    nose = 0.184
    return dict(
        x=lin("x") + nose * vx / speed,
        y=lin("y") + nose * vy / speed,
        target_x=float(b["target_x"]),
        target_y=float(b["target_y"]),
        t=lin("t"),
        speed=speed,
    )


def fit(xs, ys):
    n = len(xs)
    if n < 3:
        return float("nan"), float("nan")
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0 or syy == 0:
        return float("nan"), float("nan")
    return sxy / sxx, sxy / math.sqrt(sxx * syy)


def main():
    for path in sys.argv[1:]:
        shot_rows = shots(path)
        points = [p for p in (plane_crossing(s, float(s[0]["target_z"])) for s in shot_rows) if p]
        if not points:
            print(f"{path}: 没有过靶面的发")
            continue
        xs = [p["target_x"] for p in points]
        dx = [p["x"] for p in points]
        dy = [p["y"] for p in points]
        slope, corr = fit(xs, dx)
        tx = sum(xs) / len(xs)
        spread_t = max(xs) - min(xs)
        spread_d = max(dx) - min(dx)
        hits = sum(1 for p in points if abs(p["y"] - p["target_y"]) <= 0.033
                   and abs(p["x"] - p["target_x"]) <= 0.463)
        print(f"{path}:")
        print(f"  发数 {len(points)}  命中(瞄点口径) {hits}/{len(points)}"
              f"  过靶平均速度 {sum(p['speed'] for p in points)/len(points):.1f} m/s")
        print(f"  靶位横移跨度 {spread_t:.3f} m -> 镖横移跨度 {spread_d:.3f} m")
        print(f"  横向跟踪增益(斜率) = {slope:+.3f}   相关 = {corr:+.3f}   "
              f"(0=没制导, 1=完全跟住)")
        print(f"  竖向: 镖 y = [{min(dy):+.3f},{max(dy):+.3f}]  靶 y = {points[0]['target_y']:+.3f}")


if __name__ == "__main__":
    main()
