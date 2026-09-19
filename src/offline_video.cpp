#include "offline_video.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

OFFLINE_VIDEO::OFFLINE_VIDEO(int width, int height, double fps)
    : width_(width), height_(height), fps_(fps)
{
    fourcc_ = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    worker_ = std::thread(&OFFLINE_VIDEO::workerLoop, this);
}

OFFLINE_VIDEO::~OFFLINE_VIDEO()
{
    stop();
    stop_flag_.store(true);
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

std::string OFFLINE_VIDEO::generateFileName() const
{
    auto now = std::time(nullptr);
    std::tm* t = std::localtime(&now);
    std::ostringstream oss;
    oss << "record_" << std::put_time(t, "%Y%m%d_%H%M%S") << ".mp4";
    return oss.str();
}

bool OFFLINE_VIDEO::start()
{
    if (is_recording_.load()) return true;

    std::lock_guard<std::mutex> wlock(writer_mtx_);
    std::string filename = generateFileName();
    if (!writer_.open(filename, fourcc_, fps_, cv::Size(width_, height_))) {
        std::cerr << "[Rec] Open Failed: " << filename << std::endl;
        return false;
    }
    is_recording_.store(true);
    std::cout << "[Rec] Start: " << filename << std::endl;
    return true;
}

void OFFLINE_VIDEO::stop()
{
    if (!is_recording_.load()) return;

    // 等待队列消化，最多等 200ms
    {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait_for(lock, std::chrono::milliseconds(200),
                     [this] { return queue_.empty(); });
        while (!queue_.empty()) queue_.pop();
    }

    is_recording_.store(false);

    std::lock_guard<std::mutex> wlock(writer_mtx_);
    if (writer_.isOpened()) writer_.release();
    std::cout << "[Rec] Stop." << std::endl;
}

void OFFLINE_VIDEO::pushFrame(const cv::Mat& frame)
{
    if (!is_recording_.load() || frame.empty()) return;

    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.size() >= max_queue_size_) {
        queue_.pop();   // 丢弃最旧的帧
    }
    queue_.push(frame.clone());
    cv_.notify_one();
}

void OFFLINE_VIDEO::workerLoop()
{
    while (!stop_flag_.load()) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait(lock, [this] {
                return !queue_.empty() || stop_flag_.load();
            });

            if (stop_flag_.load() && queue_.empty()) break;
            if (queue_.empty()) continue;

            frame = queue_.front();
            queue_.pop();
        }

        if (is_recording_.load() && !frame.empty()) {
            std::lock_guard<std::mutex> wlock(writer_mtx_);
            if (writer_.isOpened()) writer_.write(frame);
        }
    }
}