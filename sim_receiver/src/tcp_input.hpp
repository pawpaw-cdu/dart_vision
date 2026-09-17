#ifndef SIM_RECEIVER_TCP_INPUT_HPP
#define SIM_RECEIVER_TCP_INPUT_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace sim_receiver {

struct SimFrame {
    cv::Mat bgr;
    uint64_t seq = 0;
    uint64_t timestamp_us = 0;
};

class TcpInput {
public:
    explicit TcpInput(std::string host, uint16_t port);

    bool connect();
    bool connected() const;
    bool sendLine(uint16_t command_port, const std::string& line);
    bool readFrame(SimFrame& frame);
    void close();

private:
    bool connectOnce();
    bool readExact(void* buffer, size_t size);

    std::string host_;
    uint16_t port_;
    int sock_ = -1;
};

} // namespace sim_receiver

#endif
