#!/usr/bin/env python3
"""按发（per-shot）严格切分的诊断：修掉 diagnose_runs.py 的两个测量缺陷。

旧脚本的缺陷（实测确认）：
  1) 命中后 launched 立刻置 0，日志里那一发的行就断了；而遥测行是按
     "sim_t_ms 就近映射到最近一行"取 z 的，命中之后的所有遥测行都会映射到
     最后一行（z≈靶面），于是被算进"终端窗口"。→ 命中轮的终端窗口被
     命中后的行灌满（ctl_active=0、found=0），指标彻底失真。
  2) 没有按发切分遥测，跨发的行会互相污染。

本脚本的做法：
  - 仿真日志：用 launched 0->1 切发，每发的 t0/t1 是仿真全局时刻。
  - 遥测：按 sim_t_ms 落进 [t0,t1] 分配到发；再用"该发的日志"做 z 判定，
    并且只取"第一次穿过终端窗口"的那一段（进入 z<target-6 到离开 z>target+0.5），
    之后的行一律丢弃（飞过头的那 13 m 不算）。
  - 每发给出：过靶面处的横向/竖向偏差、终端段的检测率/控制激活率/
    指令加速度均值/舵角均值/末端速度。

输出：逐发表（可选）+ 轮汇总 + 轮内相关（同一配置 n=15 才是干净的对照）。
"""

from __future__ import annotations

import csv
import math
import os
import sys

PLATE_HALF_HEIGHT_M = 0.033
PLATE_HALF_WIDTH_M = 0.463
NOSE_AHEAD_M = 0.184
TERMINAL_BEFORE_M = 6.0
TERMINAL_AFTER_M = 0.5


def _f(row, key, default=0.0):
    v = row.get(key)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def load(tag):
    log_path = f"flight_log_{tag}.csv"
    tel_path = f"/tmp/aim_{tag}.csv"
    if not (os.path.exists(log_path) and os.path.exists(tel_path)):
        return None
    with open(log_path) as fh:
        logs = list(csv.DictReader(fh))
    with open(tel_path) as fh:
        tel = list(csv.DictReader(fh))
    return logs, tel


def split_shots(logs):
    segments, cur = [], None
    for row in logs:
        if row["launched"] == "1":
            cur = [row] if cur is None else cur + [row]
        else:
            if cur is not None:
                segments.append(cur)
                cur = None
    if cur:
        segments.append(cur)
    return segments


def estimate_miss(shot, target_z):
    """把机心外推到靶面，再沿弹道前伸一个镖长，得到机头落点偏差。"""
    ref = None
    for a, b in zip(shot, shot[1:]):
        za, zb = _f(a, "z"), _f(b, "z")
        if za < target_z <= zb and abs(zb - za) > 1e-9:
            ref = (a, b, (target_z - za) / (zb - za))
    if ref is None:
        return None
    a, b, frac = ref
    lerp = lambda k: _f(a, k) + (_f(b, k) - _f(a, k)) * frac
    vy, vz, vx = lerp("vy"), lerp("vz"), lerp("vx")
    speed = math.hypot(vy, vz)
    if speed < 1e-6:
        return None
    return dict(
        miss_y=lerp("y") - _f(b, "target_y") + NOSE_AHEAD_M * vy / speed,
        miss_x=lerp("x") - _f(b, "target_x") + NOSE_AHEAD_M * vx / speed,
        y=lerp("y"), x=lerp("x"), vy=vy, vz=vz, vx=vx,
        speed_at_plane=math.hypot(vx, vy, vz),
        target_x=_f(b, "target_x"),
        target_y=_f(b, "target_y"),
        t_plane=_f(a, "t") + (_f(b, "t") - _f(a, "t")) * frac,
    )


def terminal_log_rows(shot, target_z):
    """第一次穿过终端窗口的那一段（离开后立刻截止）。"""
    out, entered = [], False
    for row in shot:
        z = _f(row, "z")
        if not entered:
            if z >= target_z - TERMINAL_BEFORE_M:
                entered = True
            else:
                continue
        if z > target_z + TERMINAL_AFTER_M:
            break
        out.append(row)
    return out


def analyse(tag, verbose=False):
    loaded = load(tag)
    if loaded is None:
        return None
    logs, tel = loaded
    shots = split_shots(logs)
    if not shots:
        return None
    target_z = _f(shots[0][0], "target_z")
    target_y = _f(shots[0][0], "target_y")

    # 遥测按 sim_t_ms 分配到发
    tel_by_shot = [[] for _ in shots]
    spans = [(_f(s[0], "t") * 1000.0, _f(s[-1], "t") * 1000.0) for s in shots]
    for row in tel:
        if row.get("state_valid") != "1":
            continue
        ms = _f(row, "sim_t_ms", -1.0)
        if ms < 0:
            continue
        for i, (a, b) in enumerate(spans):
            if a <= ms <= b + 120.0:   # 容忍最后一帧之后 0.12 s 的余量
                tel_by_shot[i].append(row)
                break

    per_shot = []
    for i, shot in enumerate(shots):
        miss = estimate_miss(shot, target_z)
        term = terminal_log_rows(shot, target_z)
        if miss is None or not term:
            continue
        t_lo, t_hi = _f(term[0], "t") * 1000.0, _f(term[-1], "t") * 1000.0
        tw = [r for r in tel_by_shot[i]
              if t_lo - 60.0 <= _f(r, "sim_t_ms", -1.0) <= t_hi + 60.0]
        n = len(tw)
        hit = abs(miss["miss_y"]) <= PLATE_HALF_HEIGHT_M and abs(miss["miss_x"]) <= PLATE_HALF_WIDTH_M
        per_shot.append(dict(
            idx=i + 1, hit=hit, **miss,
            n_tel=n,
            det=(sum(1 for r in tw if r.get("found") == "1") / n) if n else 0.0,
            ctl=(sum(1 for r in tw if r.get("ctl_active") == "1") / n) if n else 0.0,
            accel=(sum(_f(r, "accel_cmd_mps2") for r in tw) / n) if n else 0.0,
            avail=(sum(_f(r, "avail_accel_mps2") for r in tw) / n) if n else 0.0,
            cmd_pitch=(sum(_f(r, "ctl_pitch_deg") for r in tw) / n) if n else 0.0,
            cmd_yaw=(sum(_f(r, "ctl_yaw_deg") for r in tw) / n) if n else 0.0,
            speed=(sum(_f(r, "speed_mps") for r in tw) / n) if n else 0.0,
            fin_p=(sum(abs(_f(r, "fin1")) for r in term) / len(term)),
            fin_y=(sum(abs(_f(r, "fin2")) for r in term) / len(term)),
            t_flight=(_f(term[-1], "t") - _f(shot[0], "t")),
        ))

    if not per_shot:
        return None
    hits = sum(1 for p in per_shot if p["hit"])
    mean = lambda k: sum(p[k] for p in per_shot) / len(per_shot)

    def corr(key):
        xs = [p[key] for p in per_shot]
        ys = [p["miss_y"] for p in per_shot]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
        sy = math.sqrt(sum((y - my) ** 2 for y in ys))
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy) if sx * sy else 0.0

    if verbose:
        print(f"\n=== {tag}  靶面 {PLATE_HALF_WIDTH_M*2:.3f}x{PLATE_HALF_HEIGHT_M*2:.3f} m ===")
        print(f"{'#':>3}{'命中':>5}{'横向偏差':>10}{'竖向偏差':>10}{'靶x':>8}{'vx':>7}{'vy':>7}"
              f"{'过靶速度':>9}{'检测':>7}{'控制':>7}{'指令a':>8}{'可用a':>8}{'飞行s':>7}")
        for p in per_shot:
            print(f"{p['idx']:>3}{'✓' if p['hit'] else '×':>5}{p['miss_x']:>+10.3f}{p['miss_y']:>+10.3f}"
                  f"{p['target_x']:>+8.2f}{p['vx']:>+7.2f}{p['vy']:>+7.2f}{p['speed_at_plane']:>9.1f}"
                  f"{p['det']*100:>6.0f}%{p['ctl']*100:>6.0f}%{p['accel']:>8.1f}{p['avail']:>8.1f}"
                  f"{p['t_flight']:>7.2f}")

    return dict(tag=tag, shots=len(per_shot), hit_rate=hits / len(per_shot),
                miss_x=mean("miss_x"), miss_y=mean("miss_y"),
                sd_y=math.sqrt(sum((p["miss_y"] - mean("miss_y")) ** 2 for p in per_shot) / len(per_shot)),
                sd_x=math.sqrt(sum((p["miss_x"] - mean("miss_x")) ** 2 for p in per_shot) / len(per_shot)),
                vx=mean("vx"), vy=mean("vy"), speed=mean("speed_at_plane"),
                det=mean("det"), ctl=mean("ctl"), accel=mean("accel"), avail=mean("avail"),
                t_flight=mean("t_flight"), rows=per_shot,
                corr={(k): corr(k) for k in ("det", "ctl", "accel", "avail", "speed", "vx", "target_x")})


def main():
    tags = sys.argv[1:]
    verbose = False
    if "--verbose" in tags:
        tags.remove("--verbose")
        verbose = True
    results = [r for r in (analyse(t, verbose) for t in tags) if r]
    header = (f"{'轮次':<10}{'发':>3}{'命中':>6}{'横向均值':>9}{'竖向均值':>9}{'竖向σ':>8}"
              f"{'vx均值':>8}{'vy均值':>8}{'过靶v':>7}{'检测':>7}{'控制':>7}{'指令a':>7}{'可用a':>7}")
    print(header)
    print("-" * len(header))
    for r in sorted(results, key=lambda r: r["hit_rate"]):
        print(f"{r['tag']:<10}{r['shots']:>3}{r['hit_rate']*100:>5.0f}%{r['miss_x']:>+9.3f}{r['miss_y']:>+9.3f}"
              f"{r['sd_y']:>8.3f}{r['vx']:>+8.2f}{r['vy']:>+8.2f}{r['speed']:>7.1f}"
              f"{r['det']*100:>6.0f}%{r['ctl']*100:>6.0f}%{r['accel']:>7.1f}{r['avail']:>7.1f}")
    if len(results) == 1:
        print("\n轮内相关（n=%d 发，同一配置下的干净对照）:" % results[0]["shots"])
        for k, v in results[0]["corr"].items():
            print(f"  corr({k:<8}, 竖向偏差) = {v:+.3f}")


if __name__ == "__main__":
    main()
