#include "sim_receiver/vision_date_recevier.hpp"

#include "tcp_input.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace sim_receiver {

class VisionDateRecevier::Impl {
public:
    Impl(std::string host, uint16_t port)
        : input(std::move(host), port) {}

    void overrideEndpoint(std::string host, uint16_t port) {
        input.close();
        input = TcpInput(std::move(host), port);
    }

    bool connect() {
        return input.connect();
    }

    bool connected() const {
        return input.connected();
    }

    bool sendFireCommand() {
        return input.sendLine(7667, "FIRE\n");
    }

    bool sendGimbalCommand(double yaw, double pitch) {
        if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
            return false;
        }

        char command[96];
        std::snprintf(command, sizeof(command), "GIMBAL %.17g %.17g\n", yaw, pitch);
        return input.sendLine(7667, command);
    }

    cv::Mat getFrame() {
        SimFrame frame;
        if (!input.readFrame(frame)) {
            return cv::Mat();
        }
        return std::move(frame.bgr);
    }

    TcpInput input;
};

VisionDateRecevier::VisionDateRecevier(const std::string& config_path)
    : impl_(std::make_unique<Impl>("127.0.0.1", 7666)) {
    try {
        YAML::Node node = YAML::LoadFile(config_path);
        if (node["host"]) {
            overrideEndpoint(node["host"].as<std::string>(), 7666);
        }
        if (node["port"]) {
            overrideEndpoint(node["host"] ? node["host"].as<std::string>() : "127.0.0.1",
                             static_cast<uint16_t>(node["port"].as<int>()));
        }
    } catch (const std::exception& err) {
        std::cerr << "Failed to load simulator config: " << err.what() << std::endl;
    }
}

VisionDateRecevier::~VisionDateRecevier() = default;

void VisionDateRecevier::overrideEndpoint(const std::string& host, uint16_t port) {
    impl_->overrideEndpoint(host, port);
}

bool VisionDateRecevier::connect() {
    return impl_->connect();
}

bool VisionDateRecevier::connected() const {
    return impl_->connected();
}

bool VisionDateRecevier::sendFireCommand() {
    return impl_->sendFireCommand();
}

bool VisionDateRecevier::sendGimbalCommand(double yaw, double pitch) {
    return impl_->sendGimbalCommand(yaw, pitch);
}

cv::Mat VisionDateRecevier::getFrame() {
    return impl_->getFrame();
}

} // namespace sim_receiver
