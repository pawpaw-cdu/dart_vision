#ifndef DART_VISION_FRAME_SOURCE_HPP
#define DART_VISION_FRAME_SOURCE_HPP

#include <memory>
#include <string>

#include "dart_vision/config.hpp"
#include "dart_vision/types.hpp"

namespace dart_vision {

// 图像源抽象：一份是战队内部仿真器（TCP），一份是离线录像（便于无仿真器时验证）。
class FrameSource {
public:
    virtual ~FrameSource() {}
    virtual bool open(std::string* error) = 0;
    virtual bool next(Frame* frame) = 0;
    virtual void close() {}
    virtual std::string describe() const = 0;
    virtual double nominalFps() const { return 0.0; }
};

// source.mode == "tcp"：simulator_system 的 32 字节帧头 + JPEG
std::unique_ptr<FrameSource> createTcpFrameSource(const std::string& config_path,
                                                  const SourceConfig& cfg,
                                                  std::string* error);

// source.mode == "video"：OpenCV 读本地录像
std::unique_ptr<FrameSource> createVideoFrameSource(const SourceConfig& cfg,
                                                    std::string* error);

std::unique_ptr<FrameSource> createFrameSource(const std::string& config_path,
                                               const SourceConfig& cfg,
                                               std::string* error);

}  // namespace dart_vision

#endif  // DART_VISION_FRAME_SOURCE_HPP
