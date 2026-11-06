#!/usr/bin/env python3
"""分析 dart_vision 的遥测 CSV（--telemetry），用来调闭环制导效果。

用法：
  python3 tools/plot_aim_telemetry.py /tmp/aim.csv            # 只打印指标
  python3 tools/plot_aim_telemetry.py /tmp/aim.csv --plot     # 画曲线（需要 matplotlib）
  python3 tools/plot_aim_telemetry.py /tmp/aim.csv --launch-t 1.8   # 指定发射时刻(秒)

打印的关键指标（制导调参用）：
  * 命中率/最长连续命中：检测是否稳定（丢太久仿真器会 0.3s 超时回退比例导引）
  * 控制中断：连续未命中超过 0.3s 的段数（这些时刻外部控制会掉线）
  * 首次收敛时刻：|yaw| 与 |pitch| 同时小于阈值（默认 1°）的时刻
  * 终端 1s 平均 |误差| 与 终端 1s 平均视线角速率：核心制导指标（视线角速率趋于 0 = 收敛）
  * 误差过零次数：振荡指标，次数多说明比例增益过大、缺阻尼
"""
import argparse
import csv
import math
import sys


def load(path: str):
    rows = []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append(
                    {
                        "t": float(row["t_ms"]) / 1000.0,
                        "found": int(row["found"]),
                        "yaw": float(row["yaw_deg"]),
                        "pitch": float(row["pitch_deg"]),
                        "yaw_rate": float(row["yaw_rate_dps"]),
                        "pitch_rate": float(row["pitch_rate_dps"]),
                        "dist": float(row["dist_m"]),
                        "phase": row["phase"],
                    }
                )
            except (KeyError, ValueError):
                continue
    return rows


def summarise(rows, threshold_deg: float, launch_t: float) -> None:
    if not rows:
        print("遥测为空")
        return
    hits = [r for r in rows if r["found"]]
    total = len(rows)
    print(f"帧数 {total}，命中 {len(hits)}（{100.0 * len(hits) / total:.1f}%）")

    # 控制中断：连续未命中 > 0.3s（对应 [servo_pid] command_timeout_secs）
    gaps, gap_start = [], None
    for r in rows:
        if not r["found"] and gap_start is None:
            gap_start = r["t"]
        elif r["found"] and gap_start is not None:
            if r["t"] - gap_start > 0.3:
                gaps.append((gap_start, r["t"] - gap_start))
            gap_start = None
    if gap_start is not None and rows[-1]["t"] - gap_start > 0.3:
        gaps.append((gap_start, rows[-1]["t"] - gap_start))
    print(f"控制中断（>0.3s 未命中）: {len(gaps)} 段" + (f" {gaps[:5]}" if gaps else ""))

    window = [r for r in hits if r["t"] >= launch_t]
    if len(window) < 2:
        print("发射后的有效帧太少，无法评估收敛")
        return

    err = [max(abs(r["yaw"]), abs(r["pitch"])) for r in window]
    los = [math.hypot(r["yaw_rate"], r["pitch_rate"]) for r in window]

    converge = next((r["t"] for r, e in zip(window, err) if e < threshold_deg), None)
    if converge is None:
        print(f"未在记录内收敛到 {threshold_deg}°（最小误差 {min(err):.2f}°）")
    else:
        # 找到首次进入阈值后是否保持（简单判定：之后 80% 的帧仍在阈值内）
        idx = err.index(min(err))
        tail_ok = sum(1 for e in err[idx:] if e < threshold_deg) / max(1, len(err[idx:]))
        print(
            f"首次收敛（|yaw|,|pitch| < {threshold_deg}°）: t={converge:.2f}s，"
            f"此后 {tail_ok * 100:.0f}% 的帧在阈值内"
        )

    end_window = [r for r in window if r["t"] >= window[-1]["t"] - 1.0]
    if end_window:
        end_err = sum(max(abs(r["yaw"]), abs(r["pitch"])) for r in end_window) / len(end_window)
        end_los = sum(math.hypot(r["yaw_rate"], r["pitch_rate"]) for r in end_window) / len(
            end_window
        )
        print(f"终端 1s：平均 |误差| {end_err:.2f}°，平均视线角速率 {end_los:.2f}°/s")

    crossings = 0
    prev = None
    for r in window:
        sign = 1 if r["yaw"] >= 0 else -1
        if prev is not None and sign != prev:
            crossings += 1
        prev = sign
    print(f"yaw 误差过零次数: {crossings}（>4 次说明比例增益偏大、缺阻尼）")
    print(f"最大 |误差| {max(err):.2f}°；误差 <1° 的帧占比 {100.0 * sum(1 for e in err if e < 1.0) / len(err):.1f}%")


def plot(rows, launch_t: float) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("未安装 matplotlib，跳过绘图（pip install matplotlib）", file=sys.stderr)
        return
    hits = [r for r in rows if r["found"] and r["t"] >= launch_t]
    if not hits:
        print("发射后没有命中帧，无法绘图", file=sys.stderr)
        return
    t = [r["t"] - hits[0]["t"] for r in hits]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
    axes[0].plot(t, [r["yaw"] for r in hits], label="yaw error")
    axes[0].plot(t, [r["pitch"] for r in hits], label="pitch error")
    axes[0].axhline(0.0, color="k", lw=0.5)
    axes[0].set_ylabel("aim error [deg]")
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(t, [math.hypot(r["yaw_rate"], r["pitch_rate"]) for r in hits], color="tab:red")
    axes[1].set_ylabel("LOS rate [deg/s]")
    axes[1].grid(True)
    axes[2].plot(t, [r["dist"] for r in hits], color="tab:green")
    axes[2].set_ylabel("distance [m]")
    axes[2].set_xlabel("time since launch [s]")
    axes[2].grid(True)
    fig.suptitle("dart_vision guidance telemetry")
    fig.tight_layout()
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--threshold-deg", type=float, default=1.0)
    parser.add_argument("--launch-t", type=float, default=0.0, help="记录内发射时刻（秒）")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    rows = load(args.csv_path)
    summarise(rows, args.threshold_deg, args.launch_t)
    if args.plot:
        plot(rows, args.launch_t)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
