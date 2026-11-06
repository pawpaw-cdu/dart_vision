#!/usr/bin/env python3
"""本地假仿真器：把一段录像按 simulator_system 协议推给 dart_vision，并打印回发指令。

用途：手边没有运行中的 Rust 仿真器时，也能整条链路联调「检测 -> 解算 -> 回发」。

协议（与 simulator_system/API.md 一致）：
  * 图像流：TCP 图像端口，32 字节大端帧头 + JPEG
      magic "DAED" | version=1 | width | height | payload_len | timestamp_us(u64) | seq
  * 指令通道：TCP 指令端口，接收 dart_vision 发来的数据并打印

示例：
  python3 tools/fake_sim_server.py --video record.mp4 --fps 30
  # 另开一个终端
  ./build/dart_vision --source tcp --host 127.0.0.1 --port 7666 --command-port 7667
"""
import argparse
import socket
import struct
import sys
import threading
import time

import cv2


def frame_bytes(payload: bytes, width: int, height: int, seq: int) -> bytes:
    header = struct.pack(
        ">4sIIIIQI",
        b"DAED",
        1,
        width,
        height,
        len(payload),
        int(time.time() * 1e6),
        seq,
    )
    return header + payload


def command_listener(port: int, host: str, stop: threading.Event) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(4)
    server.settimeout(0.5)
    print(f"[cmd] 指令通道监听 {host}:{port}", flush=True)
    conn = None
    while not stop.is_set():
        try:
            if conn is None:
                conn, addr = server.accept()
                print(f"[cmd] dart_vision 已连接 {addr}", flush=True)
            data = conn.recv(4096)
            if not data:
                conn.close()
                conn = None
                continue
            if data[:1] == b"\xff" and len(data) >= 7:
                x = (data[1] << 8) | data[2]
                y = (data[3] << 8) | data[4]
                print(f"[cmd] szu_uart 帧 x={x} y={y} w={data[5]} h={data[6]}", flush=True)
            else:
                print(f"[cmd] {data.decode('utf-8', 'replace').strip()}", flush=True)
        except socket.timeout:
            continue
        except OSError:
            break
    if conn is not None:
        conn.close()
    server.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--image-port", type=int, default=7666)
    parser.add_argument("--command-port", type=int, default=7667)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--quality", type=int, default=95)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--frames", type=int, default=0, help="推送到第几帧后停止（0=不限）")
    args = parser.parse_args()

    stop = threading.Event()
    threading.Thread(
        target=command_listener, args=(args.command_port, args.host, stop), daemon=True
    ).start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.image_port))
    server.listen(1)
    print(f"[img] 图像流监听 {args.host}:{args.image_port}，等待 dart_vision 连接...", flush=True)

    conn, addr = server.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"[img] dart_vision 已连接 {addr}", flush=True)

    sent = 0
    interval = 1.0 / max(1e-3, args.fps)
    next_time = time.time()
    try:
        while not args.frames or sent < args.frames:
            capture = cv2.VideoCapture(args.video)
            if not capture.isOpened():
                print(f"[img] 无法打开 {args.video}", file=sys.stderr)
                break
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                ok, encoded = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, args.quality]
                )
                if not ok:
                    continue
                conn.sendall(
                    frame_bytes(encoded.tobytes(), frame.shape[1], frame.shape[0], sent)
                )
                sent += 1
                next_time += interval
                delay = next_time - time.time()
                if delay > 0:
                    time.sleep(delay)
                if args.frames and sent >= args.frames:
                    break
            capture.release()
            if not args.loop:
                break
    except (BrokenPipeError, ConnectionResetError, socket.timeout):
        print("[img] 客户端断开", flush=True)
    finally:
        stop.set()
        conn.close()
        server.close()
        print(f"[img] 共发送 {sent} 帧", flush=True)
        time.sleep(0.3)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
