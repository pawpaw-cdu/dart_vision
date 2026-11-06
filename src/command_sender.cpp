#include "dart_vision/command_sender.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace dart_vision {

namespace {

constexpr int kConnectTimeoutMs = 800;

// szu_uart 帧里 x/y 是 16 位无符号，越界时夹紧而不是回绕
int clampToU16(int value) { return std::max(0, std::min(65535, value)); }

// macOS 没有 MSG_NOSIGNAL，用 SO_NOSIGPIPE 达到同样效果（避免对端断开时进程被 SIGPIPE 杀掉）
int sendFlags() {
#ifdef MSG_NOSIGNAL
    return MSG_NOSIGNAL;
#else
    return 0;
#endif
}

void setNoSigPipe(int fd) {
#ifdef SO_NOSIGPIPE
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#else
    (void)fd;
#endif
}

}  // namespace

CommandSender::CommandSender(const CommandConfig& cfg, DartStateHolder* state_holder)
    : cfg_(cfg), state_holder_(state_holder) {}

CommandSender::~CommandSender() { stop(); }

void CommandSender::start() {
    if (running_.exchange(true)) return;

    // 开机握手：对应脚本里的 uart.write(0xFF + "RPDART")“视觉上线”上报。
    // sim_gimbal 模式下直接对接 simulator_system 的文本指令口，不发握手，
    // 避免模拟器打印 "Ignoring invalid simulator command"。
    if (cfg_.protocol == "szu_uart") {
        pending_.payload.assign({0xFF, 'R', 'P', 'D', 'A', 'R', 'T'});
        has_pending_ = true;
    } else if (cfg_.protocol != "sim_gimbal") {
        const std::string line = "VISION RDY\n";
        pending_.payload.assign(line.begin(), line.end());
        has_pending_ = true;
    }

    worker_ = std::thread(&CommandSender::workerLoop, this);
    // 同一条连接上还要读仿真器回推的 STATE 报文（控制律需要镖的速度/姿态）
    if (state_holder_ != nullptr) {
        reader_ = std::thread(&CommandSender::readerLoop, this);
    }
}

void CommandSender::stop() {
    if (!running_.exchange(false)) return;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (reader_.joinable()) reader_.join();
    closeSocket();
}

void CommandSender::readerLoop() {
    std::string buffer;
    char chunk[1024];
    while (running_.load()) {
        if (sock_ < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            buffer.clear();
            continue;
        }
        struct pollfd pfd;
        pfd.fd = sock_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        const int ready = poll(&pfd, 1, 100);
        if (ready <= 0) continue;
        const ssize_t n = ::recv(sock_, chunk, sizeof(chunk), 0);
        if (n <= 0) {
            // 对端关闭：交给 workerLoop 的重连逻辑处理
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        buffer.append(chunk, static_cast<std::size_t>(n));
        std::size_t newline = buffer.find('\n');
        while (newline != std::string::npos) {
            const std::string line = buffer.substr(0, newline);
            buffer.erase(0, newline + 1);
            DartState state;
            if (parseStateLine(line, &state)) state_holder_->update(state);
            newline = buffer.find('\n');
        }
        if (buffer.size() > 8192) buffer.clear();
    }
}

void CommandSender::push(Message&& message) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (has_pending_) ++dropped_;
    pending_ = std::move(message);
    has_pending_ = true;
    cv_.notify_all();
}

CommandSender::Message CommandSender::encodeAim(const AimSolution& solution,
                                                const Blob& blob) const {
    Message msg;
    const double yaw = solution.yaw_pred;
    const double pitch = solution.pitch_pred;

    if (cfg_.protocol == "sim_gimbal") {
        // simulator_system 文本指令：弧度，yaw 右正 / pitch 上正
        char buffer[128];
        std::snprintf(buffer, sizeof(buffer), "GIMBAL %.6f %.6f\n", yaw, pitch);
        const std::string line(buffer);
        msg.payload.assign(line.begin(), line.end());
        return msg;
    }

    if (cfg_.protocol == "text_angle") {
        char buffer[256];
        if (!cfg_.text_verbose) {
            // 严格三字段：guided_dart 的 parse_aim_command 只取 <axis> <yaw> <pitch>，
            // 且要求前面没有多余字段的解析器（如 parse_gimbal_command）也能兼容
            std::snprintf(buffer, sizeof(buffer), "AIM %s %.6f %.6f\n",
                          axisName(solution.axis), yaw, pitch);
        } else {
            std::snprintf(buffer, sizeof(buffer), "AIM %s %.6f %.6f %.3f %.1f %.1f %d %d %lld\n",
                          axisName(solution.axis), yaw, pitch,
                          static_cast<double>(solution.distance_m),
                          static_cast<double>(solution.u), static_cast<double>(solution.v),
                          blob.w, blob.h, static_cast<long long>(solution.t_ms));
        }
        const std::string line(buffer);
        msg.payload.assign(line.begin(), line.end());
        return msg;
    }

    // 默认（szu_uart）：复刻脚本的 7 字节帧
    //   [0xFF, x>>8, x&0xFF, y>>8, y&0xFF, w, h]
    //
    // 宽高只有 1 字节，超过 255 必须截断；但收端（仿真器/主控）是用
    // center = x + w/2 反算目标中心的，所以截断时要把 x/y 一起平移，
    // 保证「中心」不变。否则目标贴近画面边缘、或导向灯占满画面时，
    // 反算出来的中心会漂移，甚至符号相反（把目标朝反方向打）。
    int w = std::max(1, std::min(255, blob.w));
    int h = std::max(1, std::min(255, blob.h));
    const int x = clampToU16(static_cast<int>(std::lround(blob.centerX() - w * 0.5)));
    const int y = clampToU16(static_cast<int>(std::lround(blob.centerY() - h * 0.5)));
    msg.payload = {static_cast<uint8_t>(0xFF),
                   static_cast<uint8_t>((x >> 8) & 0xFF),
                   static_cast<uint8_t>(x & 0xFF),
                   static_cast<uint8_t>((y >> 8) & 0xFF),
                   static_cast<uint8_t>(y & 0xFF),
                   static_cast<uint8_t>(w),
                   static_cast<uint8_t>(h)};
    return msg;
}

void CommandSender::sendAim(const AimSolution& solution, const Blob& blob) {
    if (!running_.load() || !cfg_.enabled) return;
    push(encodeAim(solution, blob));
}

void CommandSender::sendServo(double pitch_rad, double yaw_rad, double roll_rad) {
    if (!running_.load() || !cfg_.enabled) return;
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "DART %.6f %.6f %.6f\n", pitch_rad, yaw_rad, roll_rad);
    const std::string line(buffer);
    Message msg;
    msg.payload.assign(line.begin(), line.end());
    push(std::move(msg));
}

void CommandSender::sendFire() {
    if (!running_.load() || !cfg_.enabled) return;
    Message msg;
    const std::string line = "FIRE\n";
    msg.payload.assign(line.begin(), line.end());
    push(std::move(msg));
}

bool CommandSender::connectSocket() {
    closeSocket();

    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    const std::string port = std::to_string(cfg_.port);
    if (getaddrinfo(cfg_.host.c_str(), port.c_str(), &hints, &result) != 0) {
        return false;
    }

    for (struct addrinfo* addr = result; addr != nullptr; addr = addr->ai_next) {
        int fd = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if (fd < 0) continue;

        // 非阻塞 connect + poll，避免目标不可达时长时间阻塞回发线程
        const int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        int rc = ::connect(fd, addr->ai_addr, addr->ai_addrlen);
        if (rc != 0 && errno == EINPROGRESS) {
            struct pollfd pfd;
            pfd.fd = fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            if (poll(&pfd, 1, kConnectTimeoutMs) > 0) {
                int err = 0;
                socklen_t len = sizeof(err);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0) {
                    rc = 0;
                }
            }
        }
        if (rc == 0) {
            fcntl(fd, F_SETFL, flags);  // 恢复阻塞
            int nodelay = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
            setNoSigPipe(fd);
            sock_ = fd;
            break;
        }
        ::close(fd);
    }
    freeaddrinfo(result);

    connected_.store(sock_ >= 0);
    return sock_ >= 0;
}

void CommandSender::closeSocket() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    connected_.store(false);
}

bool CommandSender::writeAll(const uint8_t* data, std::size_t size) {
    std::size_t done = 0;
    while (done < size) {
        ssize_t n = ::send(sock_, data + done, size - done, sendFlags());
        if (n > 0) {
            done += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

void CommandSender::workerLoop() {
    using clock = std::chrono::steady_clock;
    const auto interval = std::chrono::microseconds(
        static_cast<long>(cfg_.max_rate_hz > 0.0 ? 1000000.0 / cfg_.max_rate_hz : 0.0));
    auto last_send = clock::now() - interval;
    auto last_connect_attempt = clock::now() - std::chrono::hours(1);

    while (running_.load()) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait_for(lock, std::chrono::milliseconds(20),
                     [this] { return !running_.load() || has_pending_; });
        if (!running_.load()) break;
        if (!has_pending_) continue;

        const auto now = clock::now();
        if (now - last_send < interval) {
            const auto wait = interval - (now - last_send);
            lock.unlock();
            std::this_thread::sleep_for(wait);
            continue;
        }

        Message msg = std::move(pending_);
        has_pending_ = false;
        lock.unlock();

        if (sock_ < 0) {
            const auto retry = std::chrono::milliseconds(std::max(0, cfg_.reconnect_ms));
            if (now - last_connect_attempt < retry) {
                ++failed_;
                continue;
            }
            last_connect_attempt = now;
            if (!connectSocket()) {
                ++failed_;
                continue;
            }
        }

        if (writeAll(msg.payload.data(), msg.payload.size())) {
            ++sent_;
            last_send = clock::now();
        } else {
            ++failed_;
            closeSocket();
        }
    }
}

std::string CommandSender::describe() const {
    std::ostringstream oss;
    oss << cfg_.host << ":" << cfg_.port << " protocol=" << cfg_.protocol
        << " max_rate=" << cfg_.max_rate_hz << "Hz enabled=" << (cfg_.enabled ? "yes" : "no");
    return oss.str();
}

}  // namespace dart_vision
