# dart_vision

制导飞镖视觉：**接收仿真器图像 → 检测目标 → 解算角度 → 回发指令**。

图像来源是战队内部仿真器 `simulator_system`（`guided_dart` 场景）的 TCP 图像流，
检测与解算逻辑移植自深圳大学 RM2026 制导飞镖开源视觉
（`视觉代码/main_final.py`，OpenMV MicroPython 版本）。硬件（UART、舵机、
主控板）暂不接入。

```
simulator_system ──TCP:7666 (DAED 帧头 + JPEG)──▶ dart_vision
                                                      │
                          检测 LabDetector ──▶ 追踪 RoiTracker ──▶ 解算 AngleSolver
                                                      │
simulator_system ◀──TCP:7667 指令（szu_uart / text_angle / sim_gimbal）── CommandSender
```

## 1. 开源视觉的逻辑分析

`main_final.py` 是跑在 OpenMV 上的脚本，整条链路可以拆成 6 段：

1. **上电握手与模式选择**：开机先发 `0xFF + "RPDART"` 告诉主控“视觉已就绪”；
   然后阻塞等 `0x11`（yaw 模式）或 `0x22`（pitch 模式）。飞镖是滚转安装的，
   主控用这个字节告诉视觉当前该关注图像的哪个轴。
2. **传感器初始化**：QVGA(320x240) RGB565、固定曝光/增益/白平衡
   （`EXPOSURE_US=800`），并把读出窗口设成 `READOUT_ROI=(0,700,640,480)`，
   再 `set_windowing(BLOB_ROI)` 把 320x240 裁出来做检测。
3. **发射触发**：等主控的 `0xAB`（发射）→ `sensor.sleep(False)` 唤醒 → 再等
   `0x55`（非比赛）或 `0xFF`（比赛）。之后 `T_START=80ms` 内不检测。
4. **第一阶段（RECORD_TIME=900ms）**：
   `find_blobs([(9,100,-102,-25,0,127)], roi=cur_roi)` 找**绿色导向灯**，
   在候选里取**面积最大**的 blob；命中后
   `cur_roi = blob ± (FOUND_RANGE_X=40, FOUND_RANGE_Y=40)`，
   丢失后 `cur_roi` 复位成整幅 `INIT_ROI`。这就是最朴素的 ROI 自适应追踪：
   命中后只看目标附近，省算力、抗干扰。
5. **第二阶段（RECORD_TIME_PHASE2=2000ms）**：把读出窗口挪到目标所在的行区间
   （`READOUT_ROI_PHASE2 = (0, 160+blob.y*2+blob.h, 640, 480)`）。QVGA 是 2x 合并
   读出，换成这个窗口等于**用 2 倍分辨率看目标**，远距离下导向灯只有几个像素，
   这一步直接决定解算精度。
6. **回发与落盘**：`send_blob_uart()` 发 7 字节 `0xFF, x>>8, x, y>>8, y, w, h`；
   主控拿到矩形自己做导引/解算。同时按帧写 MJPEG 录像，发射结束后接收主控
   15 字节时间戳给录像改名，比赛模式下把飞行数据当 CSV 存下来。

关键结论：**开源版本里视觉端只做“检测 + 回发矩形”，角度解算和导引在主控**。
所以移植到 PC 端时，我们把解算这一环补上（下面第 3 节）。

## 2. 迁移映射

| main_final.py | dart_vision | 说明 |
|---|---|---|
| `uart.write(0xFF+"RPDART")` | `CommandSender::start()` | `szu_uart` 协议保持同样的握手字节 |
| 等 `0x11/0x22` 选 yaw/pitch | `solver.axis` | 配置文件 / `--axis` 指定 |
| `uart.any()` 等 `0xAB/0x55/0xFF` 触发 | `tracker.start_delay_ms` + 空格键 | 没有主控串口，改为上电延时 + 运行中手动上电/断电 |
| `sensor.set_auto_exposure`、`READOUT_ROI` | — | 仿真器的相机参数由 `guided_dart/config/simulator.toml` 固定，无需移植 |
| `img.find_blobs(THRESHOLDS1, roi)` | `LabDetector` | Lab 阈值语义完全保留（L 0~100，A/B -128~127），面积按**像素个数**算，与 `blob.pixels()` 一致 |
| 取面积最大的 blob | `LabDetector::detect()` | 8 邻域连通域 + 外接矩形（`blob.rect()`） |
| `cur_roi` / `FOUND_RANGE_X/Y` / `INIT_ROI` | `RoiTracker` | 命中后 ROI = 目标框外扩 ±(40,40)，连续丢失若干帧后回到整幅 |
| `T_START` | `tracker.start_delay_ms` | 起步后多久开始检测 |
| `READOUT_ROI_PHASE2` | `tracker.zoom_after_ms/zoom_factor` | 用“裁剪 + 放大”等效实现第二阶段；并额外带整幅回退，避免窗口变窄后丢帧 |
| （主控上的）PnP / 角度解算 | `AngleSolver` | `yaw=atan((u-cx)/fx)`、`pitch=atan((cy-v)/fy)`，附角速度与提前量 |
| `send_blob_uart()` | `CommandSender` | 默认 `szu_uart` 逐字节复刻；另有 `text_angle` / `sim_gimbal` |
| `send_fire`（原脚本无，主控负责） | `F` 键 / `FIRE` | 便于联调 |
| MJPEG 录像 / 飞行数据 CSV | `OFFLINE_VIDEO`（原工程自带） | 保留原实现 |

**没有迁移**：曝光/增益控制、UART 硬件、舵机与比例导引、录像改名、飞行数据 CSV
接收（都是硬件或主控侧逻辑）。

## 3. 本工程的实现

```text
include/dart_vision/
  types.hpp           Frame / Blob / Detection / AimSolution
  config.hpp          YAML 配置结构
  detector.hpp        LabDetector   ← find_max_blob()
  roi_tracker.hpp     RoiTracker    ← cur_roi + 两阶段读出窗口
  solver.hpp          AngleSolver   ← 像素→角度/距离/角速度
  command_sender.hpp  CommandSender ← send_blob_uart()（独立线程 + 只保留最新指令）
  frame_source.hpp    TCP（sim_receiver）/ 离线录像
  pipeline.hpp        串联检测→解算→回发
  overlay.hpp         draw_overlay()
src/*.cpp             上述模块实现 + main.cpp
sim_receiver/         原工程的接收库（未改动接口，仅收窄了 OpenCV 组件依赖）
tools/fake_sim_server.py  本地假仿真器，没有 Rust 仿真器时用录像联调整条链路
```

解算约定与仿真器一致：`yaw` 向右为正、`pitch` 向上为正，单位弧度。
单目测距用已知的导向灯直径（`solver.light_diameter_m=0.20`，对应
`[base.guide_light] radius = 0.10`）反推，远距离下只有像素级量化精度。

回发协议（`command.protocol`）：

| 取值 | 报文 | 用途 |
|---|---|---|
| `dart_servo`（默认） | `DART <pitch> <yaw> <roll>\n`（弧度） | **当前架构**：本工程算舵角，仿真器只执行 |
| `szu_uart` | 7 字节二进制 `0xFF, x>>8, x, y>>8, y, w, h` | 复刻"视觉发像素框、主控换算"的真实链路 |
| `text_angle` | `AIM <axis> <yaw> <pitch> ...\n` | **已废弃**：仿真器不再解释视线角 |
| `sim_gimbal` | `GIMBAL <yaw> <pitch>\n` | 手动调试：只改这两个轴 |

## 4. 配置

全部参数都在 `config/simulator.yaml`，改动说明写在文件注释里。要点：

* `host` / `port`（顶层）：`sim_receiver` 直接读取，必须保留。
* `source.mode`：`tcp`（接仿真器）或 `video`（离线回放）。
* `camera`：`guided_dart` 的机头相机是 **640x480、水平 120°**，
  对应 `fx=fy≈184.75, cx=320, cy=240`（`fov_axis=horizontal`）。
  换相机只改这一段即可。
* `detector.thresholds`：OpenMV 语义的 Lab 阈值，默认就是深圳大学的绿色导向灯。
* `detector.min_area_px`：按**像素个数**算的噪声门限（2 就是“2 个像素也算”）。
* `tracker.zoom_*`：第二阶段变焦（等效 `READOUT_ROI_PHASE2`）。
* `command.max_rate_hz`：回发限频；高帧率时旧指令会被合并丢弃。

## 5. 构建与运行

```bash
cmake -S . -B build
cmake --build build -j4

# 接仿真器（guided_dart 场景）
./build/dart_vision

# 离线回放录像调参（不依赖仿真器）
./build/dart_vision --video record_20260919_161539.mp4 --no-send

# 只检测解算、不回发；无窗口无头运行
./build/dart_vision --no-send --no-show --duration-ms 10000

# 无头构建（不链接 highgui/VideoWriter，适合飞镖机载或没有显示环境的机器）
cmake -S . -B build-headless -DDART_VISION_WITH_MEDIA=OFF
cmake --build build-headless -j4
```

常用参数：`--host/--port`（图像流）、`--command-host/--command-port`（回发）、
`--protocol`、`--axis yaw|pitch|both`、`--fast`（离线不按原帧率节流）、
`--loop/--no-loop`、`--dump-dir DIR --dump-every N`（导出画面与阈值掩码）、
`--telemetry PATH`（每帧的瞄准误差/角速度/距离写成 CSV，调制导用）、
`--log-every N`。完整列表见 `--help`。

按键（带显示构建时）：`q/ESC` 退出、`space` 上电/断电（对应原脚本的发射触发）、
`f` 发送 `FIRE`、`r` 录像开关、`z` 开关变焦、`s` 存图到 `--dump-dir`。

### 没有仿真器时怎么联调

```bash
# 终端 1：假仿真器（按 simulator_system 协议推录像，并打印回发指令）
python3 tools/fake_sim_server.py --video record_20260919_161539.mp4 --fps 30 --loop

# 终端 2
./build/dart_vision --source tcp --host 127.0.0.1 --port 7666 --command-port 7667
```

想确认"发出去的东西仿真器认不认、会被换算成多少度舵角"，用另一个工具
（按 `guided_dart/src/control.rs` 的相同规则解析，但不含物理仿真）：

```bash
# 终端 1：模拟飞镖指令口
python3 tools/fake_dart_command_server.py --port 7667 --csv /tmp/aim.csv
# 终端 2
./build/dart_vision --video record_xxx.mp4 --protocol szu_uart --command-port 7667
# 终端 1 输出：
#   [sim] szu_uart yaw=  6.875deg pitch= -5.292deg center=(430.0,303.5) wh=(36,59)
#   [sim] 解码 66 条，丢弃帧 0，非法指令 0
```

### 调制导效果（闭环）

完整的分层调参顺序（先开环核对、再内环、最后外环）见
[docs/guidance_tuning.md](docs/guidance_tuning.md)。下面是速览：

> **仿真器侧已无参数可调**：`[navigation]` 段已删除。闭环唯一的调参入口是本工程
> `config/simulator.yaml` 的 `[control]` 段。
>
> **先确认链路通**：仿真器每 0.1s 打一条 `dart pos=..., 舵角 期望/实际: pitch=..`
> 的状态行。视觉下发的舵角和这里打印的"期望"对不上就说明链路没通，
> 不要去调增益。
>
> **再分通道调**：先只调 `pn_gain`（N），再动 `aim_offset_up_m`（瞄点），
> 最后才碰 `availability_mps2` / `max_fin_deg`。详见文档第 4 节。

1. 先把靶固定住：`guided_dart/config/simulator.toml` 里
   `base.target_motion.mode = "fixed"`，把闭环当"阶跃响应"调；
   定下来之后再开 `end_moving` / `random_moving` 统计命中率。
2. 两侧同时记录数据：

   ```bash
   # 终端 1：假仿真器（收 DART，回推合成弹道的 STATE，并打印收到的舵角）
   python3 tools/fake_dart_command_server.py --port 7667 --quiet \
       --stop-after 20 --csv /tmp/cmd.csv
   # 终端 2：视觉侧（每帧瞄准误差）
   ./build/dart_vision --telemetry /tmp/aim.csv
   ```

   接真实仿真器时只跑终端 2；仿真器控制台会打印 `外部舵机控制已接管` /
   `外部舵机指令超时（无新指令），舵面回到中立`，用来确认闭环有没有掉线。
3. 分析：

   ```bash
   python3 tools/plot_aim_telemetry.py /tmp/aim.csv --launch-t 2.2 --plot
   ```

   | 指标 | 含义 |
   |---|---|
   | 控制中断（>0.3s 未命中） | 仿真器 `command_timeout_secs` 到点后舵面回中立，闭环掉线 |
   | 首次收敛时刻 | 进入 `abs(yaw),abs(pitch) < 1°` 的时刻，越小越快 |
   | 终端 1s 平均绝对误差 | 终端精度 |
   | 终端 1s 平均视线角速率 | **核心制导指标**，趋 0 说明在收敛；持续不趋 0 说明增益/提前量不对 |
   | yaw 过零次数 | 振荡指标，>4 次说明比例增益偏大、缺阻尼 |

4. 调参优先级：① 确认 `control.enabled: true`、`protocol: dart_servo`，链路通了再调；
   ② `--axis yaw|pitch` 分通道调，避免两轴耦合；③ `control.pn_gain`（N）从 1.0 起试；
   ④ `control.aim_offset_up_m` 修系统性瞄点偏差；⑤ 视觉端丢帧/限频造成的指令抖动；
   ⑥ 最后才动仿真器侧 `[servo_pid]` 与 `[servo] max_angle_deg`。

## 6. Ubuntu 部署（目标运行平台）

```bash
sudo apt update
sudo apt install -y cmake g++ libopencv-dev libyaml-cpp-dev ffmpeg
```

* `libopencv-dev` 已包含 highgui（显示窗口/录像）及其依赖；
* `ffmpeg` 只在**离线回放录像**时需要，纯 TCP 接仿真器不需要。

```bash
cmake -S . -B build          # 未指定 CMAKE_BUILD_TYPE 时默认 Release
cmake --build build -j4

./build/dart_vision                                  # 接仿真器（默认读 config/simulator.yaml）
./build/dart_vision --no-show                        # 无头运行
./build/dart_vision --video record_xxx.mp4 --no-send  # 离线调阈值

# 无显示环境（纯服务器 / 机载）建议直接构建无头版本：
cmake -S . -B build-headless -DDART_VISION_WITH_MEDIA=OFF
cmake --build build-headless -j4
```

Ubuntu/移植相关的改动，都是为了在 Linux 上不踩坑：

* 圆周率用工程内常量 `dart_vision::kPi`，不依赖 `M_PI`（glibc 在严格 ANSI 模式下
  不导出它，macOS 与 Linux 行为不一致）；
* 所有用到的标准头（`<vector>` `<cerrno>` `<cstdlib>` `<algorithm>` `<exception>` 等）
  都显式包含，不依赖传递包含；
* CMake 默认 `Release`（视觉循环对帧率敏感）；
* `highgui` 在无 `DISPLAY` 时 `imshow` 会抛异常，程序捕获后自动降级为无头运行，
  不会直接崩；按键此时不可用，改用 `--duration-ms` / Ctrl+C 退出。

Ubuntu 上联调常见问题：

* VM 用 NAT/多网卡时连不上仿真器：`config/simulator.yaml` 的 `host` 要填**仿真器所在机器**
  的 IP（不是 127.0.0.1），并确认 7666/7667 放行（`sudo ufw allow 7666/tcp`）。
* 之前用 Ubuntu VM 编的 `build/` 是 CMake 3.22 生成的，重新 `cmake -S . -B build` 即可。

## 7. 与 simulator_system 对接的注意点

* 图像流：`0.0.0.0:7666`，32 字节大端帧头（magic `DAED`、version 1、宽高、
  JPEG 长度、时间戳、序号）+ JPEG，本工程用 `sim_receiver` 接收，无需改动。
* 指令口：飞镖场景有**自己的**指令口，实现在 `guided_dart/src/control.rs`
  （`lib.rs` 里通过 `DartControlReceiver::start_from_env()` 启动），默认监听
  `0.0.0.0:7667`（可用 `DAEDALUS_COMMAND_ADDR` 改）。它和固定相机场景那套
  `simulator/src/main.rs::command_server` 是两套独立实现。

  同一条连接里可以混发二进制和文本，靠首字节区分，识别以下报文：

  | 报文 | 含义 |
  |---|---|
  | `0xFF + "RPDART"`（7B） | 上线握手，忽略 |
  | `DART <pitch> <yaw> <roll>` | **本工程正式回发**：三通道弧度舵角，直接写进舵面期望角 |
  | `SERVO <pitch> <yaw> <roll>` | 与 `DART` 等价的别名 |
  | `0xFF + x(2B 大端) + y(2B) + w(1B) + h(1B)` | 深圳大学像素框，仿真器按 `center = x + w/2` 线性换算舵角 |
  | `GIMBAL <yaw> <pitch>` | 手动调试：只改这两个轴，缺的轴保持上一次的值 |
  | `FIRE` / `VISION RDY` | 击发 / 握手（后者被忽略） |

  反向（仿真器 → 视觉）在同一条连接上按 `[vision] state_telemetry_hz`
  （默认 50Hz）回推状态：

  ```text
  STATE <t_ms> <x> <y> <z> <vx> <vy> <vz> <qw> <qx> <qy> <qz> <launched> <hit> <avail_accel>
  ```

  最后一个字段是仿真器算出的**满舵可用侧向加速度**（m/s²）。控制律拿它当
  "指令满量程"的分母，因此改仿真器 `[aero]` 舵效后不需要再手工同步
  `[control] availability_mps2`（那个值只在仿真器不回推时兜底）。

* **仿真器里已经没有制导律了。** 收到 `DART` 后它只做四片几何混合 + 舵机模型，
  控制台打印 `外部舵机控制已接管: pitch=.. yaw=.. rad`；指令带 **0.3s 超时**
  （`[servo_pid] command_timeout_secs`），超时后打印
  `外部舵机指令超时（无新指令），舵面回到中立`，舵面归零、镖按纯弹道飞。
* `AIM <axis> <yaw> <pitch>` 曾用于"视觉发视线角、仿真器做 PN"。解耦后
  **已不再被解析**（收到只会打印 `Ignoring invalid dart simulator command`）。
* 未发射、已命中时仿真器一律把舵面归零，任何外部指令都不生效。
* `FIRE` 在飞镖场景同样生效，所以本工程按 `f` 键也能触发发射。

### 现在的分工：控制全部在视觉端

相机跟着速度矢量转，画面里量到的是机体系视线角速度 θ̇，而比例导引要消的是
惯性系 λ̇ = θ̇ + γ̇，其中弹道自身的旋转率 γ̇（18 m/s 时约 31°/s）。要让视觉端
算出 λ̇，它必须同时知道**自己的速度与姿态**——这正是 `STATE` 回推的意义：

```text
图像 → 导向灯中心 (u,v) → 机体系视线角 θ_yaw / θ_pitch
     + STATE（位置 / 速度 / 四元数）
  → 机体系 → 世界系（用相机姿态，与 align_camera_to_flight 一致）
  → 单目测距（已知导向灯直径 0.20 m）
  → 灯 → 靶心几何换算（aim_offset_up_m / aim_offset_forward_m）
  → 相邻样本差分得 λ̇（跳变抑制 + 低通）
  → a = N·Vc·λ̇ + 常值偏置 + 瞄点误差反馈
  → 转回机体系 → 舵角
  → DART <pitch> <yaw> <roll> ─────────────▶ 仿真器执行
```

对应配置是 `control.enabled: true`（当前默认）。实现位置：

* `include/dart_vision/control.hpp` + `src/control.cpp`：`DartState`、
  `parseStateLine()`、`DartController`（上面整条链）；
* `src/command_sender.cpp`：读 `STATE` 的独立线程 + `sendServo()` 发 `DART`；
* `src/pipeline.cpp`：`control.enabled` 时只走这条链路，旧的角度/像素报文不再发
  （两种报文混在同一条连接上会互相干扰）。

调参入口全部在 `config/simulator.yaml` 的 `[control]` 段，见
[docs/guidance_tuning.md](docs/guidance_tuning.md)。

### 角度口径（调参前务必分清）

| 协议 | 仿真器怎么解释数值 | 特点 |
|---|---|---|
| `dart_servo`（默认） | 数值就是**弧度制期望舵角**，超过 `[servo] max_angle_deg`（默认 20°）会被限幅 | 物理口径正确；控制律在本工程里，仿真器只执行 |
| `szu_uart` | `yaw = clamp((cx-320)/320, ±1) × 20°`，`pitch = clamp((240-cy)/240, ±1) × 20°` | 像素线性映射，半幅偏移 = 满舵；复刻"视觉发像素、主控换算"时用 |
| `sim_gimbal` | 同 `dart_servo`，只是报文形式不同 | 手动调试单个轴 |

注意 `szu_uart` 走的是**另一条链路**：它不经过 `[control]` 的控制律，而是把像素框
交给仿真器做线性映射。要调闭环（测距、灯→靶心换算、λ̇、PN），必须用 `dart_servo`。

## 8. 已验证的行为

验证环境：macOS（Apple clang）+ Docker `ubuntu:22.04`
（GCC 11.4.0 / CMake 3.22.1 / OpenCV 4.5.4 / yaml-cpp 0.7.0 / ffmpeg 4.4.2）。

在 `record_20260919_161539.mp4`（640x480@30fps，4 次发射，438 帧）上实测：

| 项目 | 结果 |
|---|---|
| 检测命中 | 299/438 = 68.26%（未命中的 136 帧是复位/未发射、导向灯不可见的时段） |
| 与 Python 复现对照 | Python 用同一阈值语义得到 302/438，差异仅来自 `T_START` 的起步延时 |
| 变焦阶段 | 279 帧中 267 帧在变焦窗口内直接命中，4 帧由整幅回退救回 |
| 处理耗时 | 平均 1.4ms/帧（全幅搜索）~2.8ms/帧（变焦，含 TCP 解码），30fps 实时余量充足 |
| 端到端 TCP | 假仿真器 7666 推流 + 7667 收指令：235 次检测 → 235 条回发，0 失败 |
| 断连表现 | 指令口无人监听时 150 次发送失败，图像循环节拍仍保持 33.33ms/帧，不阻塞 |
| Ubuntu 22.04 构建 | 无头构建与带显示/录像构建均配置成功、编译通过（GCC 11.4 + OpenCV 4.5.4） |
| Ubuntu 离线回放 | 与上表完全一致（299/438，变焦窗口命中 267），平均 2.12ms/帧 |
| Ubuntu TCP 端到端 | 200 帧图像流 → 81 次检测 → 81 条回发排队、10 条实际发出、**0 失败**；测试是全速灌流（2ms/帧，远超 60Hz 限频），其余 72 条被"只保留最新指令"正常合并丢弃 |
| 与飞镖指令口契约 | 用 `tools/fake_dart_command_server.py`（复刻 `control.rs` 的解析规则）核对：`szu_uart` 66 条全部解码、0 丢帧、0 非法；`text_angle` 解码出的角度与 dart_vision 日志逐位一致 |

## 9. 已知限制

* 没有 PnP 位姿解算：只解到"机体系视线角 + 单目测距"。制导律（λ̇ + PN + 瞄点反馈、
  机体系⇄世界系变换）在本工程内完成，依赖仿真器回推的 `STATE`（位置/速度/姿态）；
  接没有状态回推的主控时只能退化成纯角度导引。
* 单目测距在目标只有 2~3 像素时基本只有“远/近”意义，需要精度就得上变焦或已知尺寸的靶面。
* `predict_horizon_ms` 是一阶线性外推，只适合目标匀速运动的补提前量。
* 硬件（UART 串口、舵机、曝光控制）没有接入，`szu_uart` 报文目前走 TCP 转发。

## 10. 环境问题排查

* **macOS + Homebrew**：如果 `ffmpeg` 升级到 9.x，而 `opencv` 4.12 是针对
  ffmpeg 8 编译的，任何链接 `opencv_videoio/highgui` 的程序都会在启动时报
  `Library not loaded: libavformat.62.dylib`。两条路：`brew reinstall opencv`，
  或者用 `-DDART_VISION_WITH_MEDIA=OFF` 构建无头版本（不链接 highgui，
  离线回放改走 `ffmpeg` 命令行管道，不受影响）。
* 离线回放需要 `ffmpeg` / `ffprobe` 在 `PATH` 中；纯 TCP 接仿真器不需要。
