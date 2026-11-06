#include "dart_vision/frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <thread>
#include <vector>

#include <sim_receiver/vision_date_receiver.hpp>

namespace dart_vision {

namespace {

int64_t nowMs() {
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count();
}

class TcpFrameSource : public FrameSource {
public:
    TcpFrameSource(const std::string& config_path, const SourceConfig& cfg)
        : cfg_(cfg), receiver_(config_path) {
        receiver_.overrideEndpoint(cfg.host, static_cast<uint16_t>(cfg.port));
    }

    bool open(std::string* error) override {
        if (!receiver_.connect()) {
            if (error) {
                *error = "无法连接仿真器图像流 " + cfg_.host + ":" + std::to_string(cfg_.port);
            }
            return false;
        }
        return true;
    }

    bool next(Frame* frame) override {
        if (!frame) return false;
        cv::Mat image = receiver_.getFrame();
        if (image.empty()) return false;  // 流关闭 / 协议错误，对应原示例的 "Frame closed!"
        frame->bgr = std::move(image);
        frame->seq = ++seq_;
        frame->timestamp_us = 0;
        frame->t_ms = nowMs();
        return true;
    }

    std::string describe() const override {
        return "tcp " + cfg_.host + ":" + std::to_string(cfg_.port);
    }

private:
    SourceConfig cfg_;
    sim_receiver::VisionDataReceiver receiver_;
    uint64_t seq_ = 0;
};

// 离线回放走 ffmpeg 命令行管道（rawvideo -> BGR），这样可以完全不依赖
// opencv_videoio：无头构建、OpenCV/ffmpeg 版本不匹配的机器上都能用。
class VideoFrameSource : public FrameSource {
public:
    explicit VideoFrameSource(const SourceConfig& cfg) : cfg_(cfg) {}

    ~VideoFrameSource() override { close(); }

    bool open(std::string* error) override {
        if (!probe(error)) return false;
        if (!openPipe(error)) return false;
        start_ = std::chrono::steady_clock::now();
        return true;
    }

    bool next(Frame* frame) override {
        if (!frame || pipe_ == nullptr) return false;

        const std::size_t need = static_cast<std::size_t>(width_) * height_ * 3;
        if (buffer_.size() != need) buffer_.assign(need, 0);

        if (std::fread(buffer_.data(), 1, need, pipe_) != need) {
            if (!cfg_.loop) return false;
            closePipe();
            std::string error;
            if (!openPipe(&error)) return false;
            if (std::fread(buffer_.data(), 1, need, pipe_) != need) return false;
        }

        frame->bgr = cv::Mat(height_, width_, CV_8UC3, buffer_.data()).clone();
        frame->seq = ++index_;
        frame->timestamp_us = 0;

        if (cfg_.realtime) {
            // 按录像原始帧率节流，保证解算/回发时序与真实接入一致
            const auto target = start_ + std::chrono::microseconds(
                                             static_cast<int64_t>(1000000.0 * index_ / fps_));
            std::this_thread::sleep_until(target);
        }
        frame->t_ms = static_cast<int64_t>(1000.0 * (index_ - 1) / std::max(1.0, fps_));
        return true;
    }

    void close() override { closePipe(); }

    std::string describe() const override {
        std::ostringstream oss;
        oss << "video " << cfg_.video_path << " " << width_ << "x" << height_ << " fps=" << fps_;
        return oss.str();
    }

    double nominalFps() const override { return fps_; }

private:
    static std::string shellQuote(const std::string& value) {
        std::string out = "'";
        for (char c : value) {
            if (c == '\'') {
                out += "'\\''";
            } else {
                out += c;
            }
        }
        out += "'";
        return out;
    }

    bool probe(std::string* error) {
        const std::string cmd = "ffprobe -v error -select_streams v:0 -show_entries "
                                "stream=width,height,r_frame_rate -of csv=p=0 " +
                                shellQuote(cfg_.video_path) + " 2>/dev/null";
        std::FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            if (error) *error = "无法执行 ffprobe，请安装 ffmpeg 或改用 TCP 图像源";
            return false;
        }
        char line[256] = {0};
        if (std::fgets(line, sizeof(line), pipe) == nullptr) {
            pclose(pipe);
            if (error) *error = "ffprobe 无法解析录像: " + cfg_.video_path;
            return false;
        }
        pclose(pipe);

        // 形如 "640,480,30/1"
        std::string text(line);
        std::replace(text.begin(), text.end(), ',', ' ');
        std::istringstream iss(text);
        std::string w, h, rate;
        iss >> w >> h >> rate;
        try {
            width_ = std::stoi(w);
            height_ = std::stoi(h);
        } catch (const std::exception&) {
            width_ = 0;
            height_ = 0;
        }
        const std::size_t slash = rate.find('/');
        if (slash != std::string::npos) {
            const double num = std::atof(rate.substr(0, slash).c_str());
            const double den = std::atof(rate.substr(slash + 1).c_str());
            fps_ = den > 0.0 ? num / den : 0.0;
        } else {
            fps_ = std::atof(rate.c_str());
        }
        if (!(fps_ > 0.0)) fps_ = 30.0;
        if (width_ <= 0 || height_ <= 0) {
            if (error) *error = "无法解析录像分辨率: " + cfg_.video_path;
            return false;
        }
        return true;
    }

    bool openPipe(std::string* error) {
        std::string cmd = "ffmpeg -nostdin -v error ";
        if (cfg_.loop) cmd += "-stream_loop -1 ";
        cmd += "-i " + shellQuote(cfg_.video_path) +
               " -an -sn -f rawvideo -pix_fmt bgr24 - 2>/dev/null";
        pipe_ = popen(cmd.c_str(), "r");
        if (!pipe_) {
            if (error) *error = "无法启动 ffmpeg，请安装 ffmpeg 或改用 TCP 图像源";
            return false;
        }
        start_ = std::chrono::steady_clock::now();
        index_ = 0;
        return true;
    }

    void closePipe() {
        if (pipe_) {
            pclose(pipe_);
            pipe_ = nullptr;
        }
    }

    SourceConfig cfg_;
    std::FILE* pipe_ = nullptr;
    std::vector<uint8_t> buffer_;
    int width_ = 0;
    int height_ = 0;
    double fps_ = 30.0;
    uint64_t index_ = 0;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace

std::unique_ptr<FrameSource> createTcpFrameSource(const std::string& config_path,
                                                  const SourceConfig& cfg,
                                                  std::string* error) {
    std::unique_ptr<FrameSource> source(new TcpFrameSource(config_path, cfg));
    if (!source->open(error)) return nullptr;
    return source;
}

std::unique_ptr<FrameSource> createVideoFrameSource(const SourceConfig& cfg,
                                                    std::string* error) {
    std::unique_ptr<FrameSource> source(new VideoFrameSource(cfg));
    if (!source->open(error)) return nullptr;
    return source;
}

std::unique_ptr<FrameSource> createFrameSource(const std::string& config_path,
                                               const SourceConfig& cfg,
                                               std::string* error) {
    if (cfg.mode == "video") return createVideoFrameSource(cfg, error);
    return createTcpFrameSource(config_path, cfg, error);
}

}  // namespace dart_vision
