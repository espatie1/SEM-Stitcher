#pragma once
#include <semstitch/core/Frame.hpp>
#include <deque>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <chrono>

struct RecvFrame {
    std::vector<std::uint8_t> buf;
    std::uint32_t width{0}, height{0};
    semstitch::PixelFormat fmt{semstitch::PixelFormat::Gray8};
    std::uint64_t ts_ns{0};
    std::chrono::steady_clock::time_point t_arrive{};
    std::size_t bytes() const {
        switch (fmt) {
            case semstitch::PixelFormat::Gray8:  return std::size_t(width) * height;
            case semstitch::PixelFormat::RGB24:  return std::size_t(width) * height * 3;
            case semstitch::PixelFormat::RGBA32: return std::size_t(width) * height * 4;
            default: return 0;
        }
    }
};

class JitterBuffer {
public:
    enum class DropPolicy { Oldest, Newest };
    JitterBuffer(std::size_t capacity, DropPolicy drop) : cap_(capacity), drop_(drop) {}

    void push(RecvFrame&& rf, std::atomic<std::uint64_t>& drops) {
        std::lock_guard<std::mutex> lk(m_);
        if (q_.size() >= cap_) {
            if (drop_ == DropPolicy::Oldest) q_.pop_front();
            else { ++drops; return; }
            ++drops;
        }
        q_.push_back(std::move(rf));
        cv_.notify_one();
    }

    bool pop_wait(RecvFrame& out, int wait_ms) {
        std::unique_lock<std::mutex> lk(m_);
        if (q_.empty()) {
            if (cv_.wait_for(lk, std::chrono::milliseconds(wait_ms)) == std::cv_status::timeout)
                return false;
        }
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

private:
    std::size_t cap_;
    DropPolicy drop_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<RecvFrame> q_;
};
