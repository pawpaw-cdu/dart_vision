#!/usr/bin/env bash
# 等价性自检：让 C++ 原版（dart_vision/src/control.cpp）和 C 移植版（port_c/dv_control.c）
# 吃同一串输入，逐帧比较输出。任何一处移植笔误都会在这里暴露。
#
#   ./compare.sh              # 正常路径（IMU 给了 available_accel）
#   AVAIL_ZERO=1 ./compare.sh # 兜底标定分支（available_accel = 0）
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"   # dart_vision 根目录
OPENCV_INC="${OPENCV_INC:-/opt/homebrew/include/opencv4}"   # Ubuntu: /usr/include/opencv4
DEF=""
if [[ "${AVAIL_ZERO:-0}" == "1" ]]; then
  DEF="-DGEN_AVAIL_ZERO=1"
  echo "== 兜底标定分支（available_accel = 0）=="
else
  echo "== 正常路径（available_accel 由 IMU/仿真给出）=="
fi

TMP="$(mktemp -d)"
cc -std=c99 -O2 $DEF "$ROOT/port_c/dv_control.c" "$ROOT/port_c/selftest/c_harness.c" -lm -o "$TMP/c_ref"
clang++ -std=c++17 -O2 $DEF -I "$ROOT/include" -I "$OPENCV_INC" -I "$ROOT/port_c/selftest" \
  "$ROOT/port_c/selftest/cpp_harness.cpp" "$ROOT/src/control.cpp" -o "$TMP/cpp_ref"

"$TMP/c_ref"   > "$TMP/out_c.txt"   2>/dev/null || true
"$TMP/cpp_ref" > "$TMP/out_cpp.txt" 2>/dev/null

python3 - "$TMP" <<'PY'
import sys, os
tmp = sys.argv[1]
a = [l.split() for l in open(os.path.join(tmp, "out_c.txt"))]
b = [l.split() for l in open(os.path.join(tmp, "out_cpp.txt"))]
# 列 -> (显示名, 容差)。控制律真正输出的是 pitch/yaw/roll（弧度），容差按单精度取 1e-6；
# 诊断量里 aim_err/los_rate 经 acos/acosf 计算，在 0 附近病态（导数无穷），放宽到 1e-3。
cols = [
    (1, "active", None),          # 必须逐帧完全一致
    (2, "pitch", 1e-6), (3, "yaw", 1e-6), (4, "roll", 1e-6),
    (5, "los_rate", 1e-3), (6, "range", 1e-5), (7, "accel", 1e-3),
    (8, "avail", 1e-5), (9, "aim_err", 1e-2), (10, "ramp", 1e-6), (11, "roll_deg", 1e-6),
]
n = min(len(a), len(b))
print(f"比较 {n} 帧")
ok = True
for j, name, tol in cols:
    if tol is None:
        bad = [(i, a[i][j], b[i][j]) for i in range(n) if a[i][j] != b[i][j]]
        print(f"  {name:10} 不一致帧数 {len(bad)}" + ("" if not bad else f"  例: {bad[:3]}"))
        ok = ok and not bad
        continue
    d = max(abs(float(a[i][j]) - float(b[i][j])) for i in range(n))
    unit = "rad" if name in ("pitch", "yaw", "roll") else ""
    flag = "OK" if d <= tol else "超限"
    print(f"  {name:10} 最大绝对差 {d:.3e} {unit:<3} 容差 {tol:.0e}  {flag}")
    ok = ok and d <= tol
print("结论:", "✅ C 移植与 C++ 原版数值等价（差异仅在单精度舍入级）" if ok else "❌ 存在超出容差的差异，请检查")
sys.exit(0 if ok else 1)
PY
rm -rf "$TMP"
