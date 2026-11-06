#ifndef DART_VISION_COMMAND_SENDER_HPP
#define DART_VISION_COMMAND_SENDER_HPP

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dart_vision/config.hpp"
#include "dart_vision/control.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 对应 main_final.py 的 send_blob_uart()：把检测结果回发给主控。
// 这里把串口换成 TCP，并放到独立线程里，避免回发阻塞图像循环。
// 高帧率时只保留最新一条指令（与 Ultra_Vision 的云台指令合并策略一致）。
class CommandSender {
public:
    CommandSender(const CommandConfig& cfg, DartStateHolder* state_holder = nullptr);
    ~CommandSender();

    CommandSender(const CommandSender&) = delete;
    CommandSender& operator=(const CommandSender&) = delete;

    void start();
    void stop();

    void sendAim(const AimSolution& solution, const Blob& blob);
    /// 直接回发三轴舵角（弧度），对应仿真器的 DART 报文
    void sendServo(double pitch_rad, double yaw_rad, double roll_rad);
    void sendFire();

    bool connected() const { return connected_.load(); }
    uint64_t sent() const { return sent_.load(); }
    uint64_t dropped() const { return dropped_.load(); }
    uint64_t failed() const { return failed_.load(); }
    std::string describe() const;

private:
    struct Message {
        std::vector<uint8_t> payload;
    };

    void workerLoop();
    void readerLoop();
    bool connectSocket();
    void closeSocket();
    bool writeAll(const uint8_t* data, std::size_t size);
    void push(Message&& message);
    Message encodeAim(const AimSolution& solution, const Blob& blob) const;

    CommandConfig cfg_;
    std::thread worker_;
    std::thread reader_;
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    Message pending_;
    bool has_pending_ = false;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> sent_{0};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<uint64_t> failed_{0};
    int sock_ = -1;
    DartStateHolder* state_holder_ = nullptr;
};

}  // namespace dart_vision

#endif  // DART_VISION_COMMAND_SENDER_HPP
