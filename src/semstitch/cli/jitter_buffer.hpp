#pragma once
#include <semstitch/core/Frame.hpp>
#include <deque>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <chrono>

/*
  A received frame container + timing info for jitter buffering.
  The pixel data is stored as raw bytes in 'buf'.
*/
struct RecvFrame {
    std::vector<std::uint8_t> buf;             // raw pixel bytes
    std::uint32_t width{0}, height{0};         // image size in pixels
    semstitch::PixelFormat fmt{semstitch::PixelFormat::Gray8}; // pixel format
    std::uint64_t ts_ns{0};                    // source timestamp (nanoseconds)
    std::chrono::steady_clock::time_point t_arrive{}; // local arrival time

    // Estimated payload size in bytes for the current format and size.
    std::size_t bytes() const {
        switch (fmt) {
            case semstitch::PixelFormat::Gray8:  return std::size_t(width) * height;
            case semstitch::PixelFormat::RGB24:  return std::size_t(width) * height * 3;
            case semstitch::PixelFormat::RGBA32: return std::size_t(width) * height * 4;
            default: return 0;
        }
    }
};

/*
  Simple thread-safe jitter buffer for streaming frames.

  - Capacity-limited queue with a drop policy:
      * Oldest: drop the oldest frame when the queue is full.
      * Newest: drop the new incoming frame instead.
  - 'drops' counter (atomic) is increased whenever a frame is dropped.
*/
class JitterBuffer {
public:
    enum class DropPolicy { Oldest, Newest };

    // Create a jitter buffer with a fixed capacity and a drop policy.
    JitterBuffer(std::size_t capacity, DropPolicy drop) : cap_(capacity), drop_(drop) {}

    // Push a new frame into the buffer.
    // If full:
    //   - Oldest: remove front (oldest), then push the new one; increment 'drops'.
    //   - Newest: reject the new frame and increment 'drops'.
    // Notifies one waiting consumer.
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

    // Pop a frame from the buffer, waiting up to 'wait_ms' milliseconds.
    // Returns true if a frame was popped, false on timeout or if still empty.
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

    // Current number of frames in the buffer (thread-safe).
    std::size_t size() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

private:
    std::size_t cap_;                 // maximum number of frames
    DropPolicy drop_;                 // drop strategy when full
    mutable std::mutex m_;            // protects the queue
    std::condition_variable cv_;      // for producer/consumer signaling
    std::deque<RecvFrame> q_;         // underlying FIFO queue
};
