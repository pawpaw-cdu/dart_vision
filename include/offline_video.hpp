#ifndef OFFLINE_VIDEO_HPP
#define OFFLINE_VIDEO_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <ctime>

class OFFLINE_VIDEO
{
public:
    // 宽、高、帧率
    OFFLINE_VIDEO(int width, int height, double fps = 60.0);
    ~OFFLINE_VIDEO();

    // 禁止拷贝（线程和 writer 不能复制）
    OFFLINE_VIDEO(const OFFLINE_VIDEO&) = delete;
    OFFLINE_VIDEO& operator=(const OFFLINE_VIDEO&) = delete;

    // 手动开始 / 停止（线程安全）
    bool start();
    void stop();
    bool isRecording() const { return is_recording_.load(); }

    // 主线程调用：非阻塞，内部 clone 后入队
    void pushFrame(const cv::Mat& frame);

private:
    void workerLoop();
    std::string generateFileName() const;

    // 配置
    int width_;
    int height_;
    double fps_;
    int fourcc_;
    size_t max_queue_size_ = 30;   // 最多缓存 30 帧

    // 线程与队列
    std::thread worker_;
    std::atomic<bool> is_recording_{false};
    std::atomic<bool> stop_flag_{false};

    std::queue<cv::Mat> queue_;
    mutable std::mutex mtx_;
    std::condition_variable cv_;

    // 保护 VideoWriter 的 open / release / write
    std::mutex writer_mtx_;
    cv::VideoWriter writer_;
};

#endif