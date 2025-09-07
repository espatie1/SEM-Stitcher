#include "semstitch/io/GrpcServer.hpp"
#include "semstitch.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <zlib.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace semstitch {

//------------------------------------------------------------------------------
// Small helpers (time, CRC, enum mapping)
//------------------------------------------------------------------------------
namespace {
static std::uint64_t to_ns(std::chrono::steady_clock::time_point tp) {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(tp.time_since_epoch()).count();
}
static std::uint32_t crc32_bytes(const void* data, std::size_t size) {
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, reinterpret_cast<const Bytef*>(data), static_cast<uInt>(size));
    return static_cast<std::uint32_t>(crc);
}
static PixelFmtPB to_pb(PixelFormat f) {
    switch (f) {
        case PixelFormat::Gray8:  return PixelFmtPB::GRAY8;
        case PixelFormat::RGB24:  return PixelFmtPB::RGB24;
        case PixelFormat::RGBA32: return PixelFmtPB::RGBA32;
        default:                  return PixelFmtPB::GRAY8;
    }
}
} // namespace

//------------------------------------------------------------------------------
// gRPC streaming server implementation
//  - keeps a bounded queue of frames (backpressure by drop policy)
//  - serves a bidi stream; we only write, but still drain inbound to be safe
//  - sends heartbeats when there are no frames
//------------------------------------------------------------------------------
class GrpcServer::Impl : public SemStitch::Service {
public:
    Impl(std::uint16_t port, Options opt)
        : opt_{opt}
        , addr_("0.0.0.0:" + std::to_string(port))
    {
        grpc::ServerBuilder b;
        b.AddListeningPort(addr_, grpc::InsecureServerCredentials());
        b.RegisterService(this);
        server_ = b.BuildAndStart();
        std::cout << "[grpc-server] listening at " << addr_ << "\n";

        // Simple monitor thread: prints queue size every second
        running_ = true;
        mon_ = std::thread([this]{
            using namespace std::chrono_literals;
            while (running_) {
                {
                    std::lock_guard<std::mutex> lk(m_);
                    if (!queue_.empty())
                        std::cout << "[grpc-server] queue=" << queue_.size() << "\n";
                }
                std::this_thread::sleep_for(1000ms);
            }
        });
    }

    ~Impl() override {
        running_ = false;
        cv_.notify_all();
        if (server_) server_->Shutdown();
        if (mon_.joinable()) mon_.join();
    }

    // Main streaming RPC. We continuously write FrameChunk messages.
    // If no data is available, we write a heartbeat chunk (width=height=0).
    ::grpc::Status StreamFrames(::grpc::ServerContext* ctx,
                                ::grpc::ServerReaderWriter<FrameChunk, FrameChunk>* stream) override
    {
        using namespace std::chrono;

        // Drain inbound messages (if any) to avoid flow-control stalls
        std::atomic<bool> drain_running{true};
        std::thread drain([&]{
            FrameChunk dummy;
            while (drain_running && stream->Read(&dummy)) {
                // ignore inbound if any
            }
        });

        while (running_ && !ctx->IsCancelled()) {
            QItem item;
            bool has_item = false;

            {
                std::unique_lock<std::mutex> lk(m_);
                if (queue_.empty()) {
                    cv_.wait_for(lk, milliseconds(opt_.heartbeatMs));
                }
                if (!queue_.empty()) {
                    item = std::move(queue_.front());
                    queue_.pop_front();
                    has_item = true;
                }
            }

            FrameChunk chunk;

            if (has_item) {
                // Fill the chunk with frame data and metadata
                chunk.set_width(item.w);
                chunk.set_height(item.h);
                chunk.set_format(to_pb(item.fmt));
                chunk.set_timestamp_ns(item.ts_ns);

                if (!item.data.empty()) {
                    chunk.set_data(reinterpret_cast<const char*>(item.data.data()), item.data.size());
                    const auto seq = seq_.fetch_add(1, std::memory_order_relaxed) + 1;
                    chunk.set_seq(seq);
                    chunk.set_crc32(crc32_bytes(item.data.data(), item.data.size()));
                } else {
                    // Empty data frame (should be rare)
                    chunk.clear_data();
                    chunk.set_seq(seq_.load(std::memory_order_relaxed));
                    chunk.set_crc32(0);
                }

                if (!stream->Write(chunk)) break;

            } else {
                // Heartbeat: keep the stream alive during idle periods
                chunk.set_width(0);
                chunk.set_height(0);
                chunk.set_format(PixelFmtPB::GRAY8);
                chunk.set_timestamp_ns(0);
                chunk.clear_data();
                chunk.set_seq(seq_.load(std::memory_order_relaxed));
                chunk.set_crc32(0);

                stream->Write(chunk);
            }
        }

        drain_running = false;
        if (drain.joinable()) drain.join();
        return ::grpc::Status::OK;
    }

    // Push a new frame into the bounded queue (thread-safe).
    // If the queue is full, apply the selected drop policy.
    void push(const Frame& f) {
        QItem q;
        q.w     = f.width;
        q.h     = f.height;
        q.fmt   = f.format;
        q.ts_ns = to_ns(f.timestamp);

        const std::size_t need = f.bytes();
        q.data.resize(need);
        if (need) std::memcpy(q.data.data(), f.data.data(), need);

        {
            std::lock_guard<std::mutex> lk(m_);
            if (static_cast<int>(queue_.size()) >= opt_.maxQueue) {
                if (opt_.dropOldest && !queue_.empty()) queue_.pop_front();
                else return; // drop newest
            }
            queue_.push_back(std::move(q));
        }
        cv_.notify_one();
    }

private:
    struct QItem {
        std::vector<std::uint8_t> data;
        std::uint32_t w{0}, h{0};
        PixelFormat   fmt{PixelFormat::Gray8};
        std::uint64_t ts_ns{0};
    };

    Options opt_;
    std::string addr_;
    std::unique_ptr<grpc::Server> server_;

    std::mutex m_;
    std::condition_variable cv_;
    std::deque<QItem> queue_;

    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> seq_{0};

    std::thread mon_;
};

GrpcServer::GrpcServer(std::uint16_t port)
    : p_(std::make_unique<Impl>(port, Options{})) {}

GrpcServer::GrpcServer(std::uint16_t port, Options opt)
    : p_(std::make_unique<Impl>(port, opt)) {}

GrpcServer::~GrpcServer() = default;

void GrpcServer::pushFrame(const Frame& f) { p_->push(f); }

} // namespace semstitch
