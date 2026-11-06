#!/usr/bin/env python3
"""按 guided_dart/src/control.rs 的规则解析 dart_vision 的回发，用来验证闭环契约。

它复刻仿真器侧的解析逻辑（端口、报文体、像素->舵角映射、超时语义），
但没有物理仿真，所以可以在不启动 Rust 仿真器的情况下回答：
  * 我发的字节/文本，仿真器到底认不认？
  * 认了之后换算成多少度舵角？符号对不对？
  * 有没有被丢弃的帧（例如 w/h 为 0）？

用法：
  python3 tools/fake_dart_command_server.py --port 7667 --protocol szu_uart
  python3 tools/fake_dart_command_server.py --port 7667 --protocol szu_uart --csv /tmp/aim.csv
  # 另开终端
  ./build/dart_vision --video record.mp4 --protocol szu_uart --command-port 7667
"""
import argparse
import math
import socket
import sys
import threading
import time

SZU_UART_HANDSHAKE = b"\xffRPDART"
SZU_UART_FRAME_SIZE = 7


class Decoder:
    """对应 control.rs 的 PixelCommandMapping / handle_ascii_command。"""

    def __init__(self, frame_width: float, frame_height: float, max_angle_deg: float):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.max_angle = math.radians(max_angle_deg)
        self.decoded = 0
        self.rejected = 0
        self.invalid = 0

    def from_szu_uart(self, frame: bytes):
        if len(frame) != SZU_UART_FRAME_SIZE or frame[0] != 0xFF:
            return None
        x = int.from_bytes(frame[1:3], "big")
        y = int.from_bytes(frame[3:5], "big")
        width = frame[5]
        height = frame[6]
        if width <= 0 or height <= 0:
            self.rejected += 1
            return None
        center_x = x + width * 0.5
        center_y = y + height * 0.5
        half_w = max(self.frame_width * 0.5, 1.0)
        half_h = max(self.frame_height * 0.5, 1.0)
        yaw = max(-1.0, min(1.0, (center_x - half_w) / half_w)) * self.max_angle
        pitch = max(-1.0, min(1.0, (half_h - center_y) / half_h)) * self.max_angle
        self.decoded += 1
        return ("szu_uart", center_x, center_y, width, height, yaw, pitch)

    def parse_ascii(self, line: str):
        parts = line.split()
        if not parts:
            return None
        tag = parts[0].upper()
        try:
            if tag in ("DART", "SERVO") and len(parts) == 4:
                pitch, yaw, roll = (float(v) for v in parts[1:4])
                self.decoded += 1
                return ("DART", None, None, None, None, yaw, pitch, roll)
            if tag == "GIMBAL" and len(parts) == 3:
                yaw, pitch = float(parts[1]), float(parts[2])
                self.decoded += 1
                return ("GIMBAL", None, None, None, None, yaw, pitch)
            if tag == "AIM" and len(parts) >= 4:
                axis, yaw, pitch = parts[1].lower(), float(parts[2]), float(parts[3])
                if axis not in ("both", "yaw", "pitch"):
                    self.invalid += 1
                    return None
                self.decoded += 1
                return ("AIM", None, None, None, None, yaw, pitch, axis)
            if tag == "FIRE":
                self.decoded += 1
                return ("FIRE", None, None, None, None, None, None)
            if line.strip().upper() == "VISION RDY":
                return None
        except ValueError:
            pass
        self.invalid += 1
        print(f"[sim] Ignoring invalid dart simulator command: {line!r}", flush=True)
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7667)
    parser.add_argument("--frame-width", type=float, default=640.0)
    parser.add_argument("--frame-height", type=float, default=480.0)
    parser.add_argument("--max-angle-deg", type=float, default=20.0)
    parser.add_argument("--csv", default="", help="把解码结果写成 CSV 便于调参")
    parser.add_argument("--quiet", action="store_true", help="只在结束时打印统计")
    parser.add_argument(
        "--stop-after", type=float, default=0.0, help="运行 N 秒后退出（0 = 一直跑）"
    )
    args = parser.parse_args()

    decoder = Decoder(args.frame_width, args.frame_height, args.max_angle_deg)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(4)
    print(
        f"[sim] guided dart 指令口监听 {args.host}:{args.port} "
        f"({int(args.frame_width)}x{int(args.frame_height)}, 满舵 {args.max_angle_deg}deg)",
        flush=True,
    )

    # 行缓冲，配合 tail -f 边跑边看
    csv_file = open(args.csv, "w", buffering=1) if args.csv else None
    if csv_file:
        csv_file.write("t_s,tag,center_x,center_y,w,h,yaw_deg,pitch_deg\n")
    t0 = time.time()
    stop = threading.Event()

    if args.stop_after > 0:
        def _auto_stop() -> None:
            time.sleep(args.stop_after)
            stop.set()

        threading.Thread(target=_auto_stop, daemon=True).start()

    def handle(conn: socket.socket) -> None:
        buffer = bytearray()
        while not stop.is_set():
            try:
                chunk = conn.recv(1024)
            except socket.timeout:
                continue  # 目标丢失时空闲是正常的，保持连接（仿真器也是常连）
            except OSError:
                break
            if not chunk:
                break
            buffer.extend(chunk)
            while buffer:
                if buffer[0] == 0xFF:
                    if len(buffer) < SZU_UART_FRAME_SIZE:
                        break
                    frame = bytes(buffer[:SZU_UART_FRAME_SIZE])
                    del buffer[:SZU_UART_FRAME_SIZE]
                    if frame == SZU_UART_HANDSHAKE:
                        continue
                    result = decoder.from_szu_uart(frame)
                    if result is None:
                        continue
                else:
                    newline = buffer.find(b"\n")
                    if newline < 0:
                        break
                    line = bytes(buffer[:newline]).decode("utf-8", "replace")
                    del buffer[: newline + 1]
                    result = decoder.parse_ascii(line)
                    if result is None:
                        continue

                tag, cx, cy, w, h, yaw, pitch = result[:7]
                if csv_file:
                    csv_file.write(
                        f"{time.time() - t0:.3f},{tag},"
                        f"{'' if cx is None else f'{cx:.1f}'},"
                        f"{'' if cy is None else f'{cy:.1f}'},"
                        f"{'' if w is None else w},{'' if h is None else h},"
                        f"{math.degrees(yaw):.3f},{math.degrees(pitch):.3f}\n"
                    )
                if not args.quiet:
                    extra = ""
                    if cx is not None:
                        extra = f" center=({cx:.1f},{cy:.1f}) wh=({w},{h})"
                    print(
                        f"[sim] {tag:8s} yaw={math.degrees(yaw):7.3f}deg "
                        f"pitch={math.degrees(pitch):7.3f}deg{extra}",
                        flush=True,
                    )

    connections = []
    try:
        server.settimeout(1.0)
        while not stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print(f"[sim] dart_vision 已连接 {addr}", flush=True)
            conn.settimeout(0.5)
            connections.append(conn)
            # 每个连接一个线程：dart_vision 断线重连时不会互相影响
            threading.Thread(target=handle, args=(conn,), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for conn in connections:
            try:
                conn.close()
            except OSError:
                pass
        server.close()
        if csv_file:
            csv_file.close()
        print(
            f"[sim] 解码 {decoder.decoded} 条，丢弃帧 {decoder.rejected}，"
            f"非法指令 {decoder.invalid}",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
