#include "tcp_input.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <utility>

namespace sim_receiver {

namespace {

constexpr uint32_t kMagic = 0x44414544; // "DAED"
constexpr uint32_t kVersion = 1;
constexpr size_t kHeaderSize = 32;

uint32_t readU32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

uint64_t readU64(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value = (value << 8) | p[i];
    }
    return value;
}

} // namespace

TcpInput::TcpInput(std::string host, uint16_t port)
    : host_(std::move(host)), port_(port) {}

bool TcpInput::connect() {
    close();
    return connectOnce();
}

bool TcpInput::connected() const {
    return sock_ >= 0;
}

bool TcpInput::sendLine(uint16_t command_port, const std::string& line) {
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    std::string port = std::to_string(command_port);
    if (getaddrinfo(host_.c_str(), port.c_str(), &hints, &result) != 0) {
        return false;
    }

    int fd = -1;
    for (struct addrinfo* addr = result; addr != nullptr; addr = addr->ai_next) {
        fd = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if (fd < 0) {
            continue;
        }
        if (::connect(fd, addr->ai_addr, addr->ai_addrlen) == 0) {
            break;
        }
        ::close(fd);
        fd = -1;
    }
    freeaddrinfo(result);

    if (fd < 0) {
        return false;
    }

    size_t sent = 0;
    while (sent < line.size()) {
        ssize_t n = ::send(fd, line.data() + sent, line.size() - sent, 0);
        if (n <= 0) {
            ::close(fd);
            return false;
        }
        sent += static_cast<size_t>(n);
    }

    ::close(fd);
    return true;
}

bool TcpInput::connectOnce() {
    close();

    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* result = nullptr;
    std::string port = std::to_string(port_);
    if (getaddrinfo(host_.c_str(), port.c_str(), &hints, &result) != 0) {
        return false;
    }

    for (struct addrinfo* addr = result; addr != nullptr; addr = addr->ai_next) {
        sock_ = ::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if (sock_ < 0) {
            continue;
        }
        if (::connect(sock_, addr->ai_addr, addr->ai_addrlen) == 0) {
            break;
        }
        ::close(sock_);
        sock_ = -1;
    }
    freeaddrinfo(result);

    if (sock_ < 0) {
        return false;
    }

    int nodelay = 1;
    setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    return true;
}

bool TcpInput::readExact(void* buffer, size_t size) {
    if (sock_ < 0 && !connectOnce()) {
        return false;
    }

    uint8_t* out = static_cast<uint8_t*>(buffer);
    size_t done = 0;
    while (done < size) {
        ssize_t n = ::recv(sock_, out + done, size - done, 0);
        if (n == 0) {
            return false;
        }
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        done += static_cast<size_t>(n);
    }
    return true;
}

bool TcpInput::readFrame(SimFrame& frame) {
    if (sock_ < 0 && !connectOnce()) {
        return false;
    }

    std::vector<uint8_t> header(kHeaderSize);
    if (!readExact(header.data(), header.size())) {
        close();
        return false;
    }

    uint32_t magic = readU32(header.data());
    uint32_t version = readU32(header.data() + 4);
    uint32_t width = readU32(header.data() + 8);
    uint32_t height = readU32(header.data() + 12);
    uint32_t payload_len = readU32(header.data() + 16);
    uint64_t timestamp_us = readU64(header.data() + 20);
    uint32_t seq = readU32(header.data() + 28);

    if (magic != kMagic || version != kVersion || payload_len == 0 ||
        payload_len > 20 * 1024 * 1024 || width == 0 || height == 0) {
        close();
        return false;
    }

    std::vector<uint8_t> payload(payload_len);
    if (!readExact(payload.data(), payload.size())) {
        close();
        return false;
    }

    cv::Mat decoded = cv::imdecode(payload, cv::IMREAD_COLOR);
    if (decoded.empty()) {
        return false;
    }

    frame.bgr = decoded;
    frame.seq = seq;
    frame.timestamp_us = timestamp_us;
    return true;
}

void TcpInput::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

} // namespace sim_receiver
