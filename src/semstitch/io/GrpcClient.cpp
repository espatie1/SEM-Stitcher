#include "semstitch/io/GrpcClient.hpp"
#include "semstitch.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <zlib.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace semstitch {

namespace {
/* Compute CRC32 over a byte buffer using zlib. */
static std::uint32_t crc32_bytes(const void* data, std::size_t size) {
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, reinterpret_cast<const Bytef*>(data), static_cast<uInt>(size));
    return static_cast<std::uint32_t>(crc);
}

/* Map protobuf pixel format to internal enum. */
static PixelFormat from_pb(PixelFmtPB f) {
    switch (f) {
        case PixelFmtPB::GRAY8:  return PixelFormat::Gray8;
        case PixelFmtPB::RGB24:  return PixelFormat::RGB24;
        case PixelFmtPB::RGBA32: return PixelFormat::RGBA32;
        default:                 return PixelFormat::Gray8;
    }
}
} // namespace

class GrpcClient::Impl {
public:
    Impl(const std::string& addr, Options opt)
        : serverAddr_{addr}, opt_{opt}
    {
        channel_ = grpc::CreateChannel(serverAddr_, grpc::InsecureChannelCredentials());
        stub_ = SemStitch::NewStub(channel_);
    }

    void start(FrameHandler cb) {
        cb_ = std::move(cb);
        running_ = true;
        worker_  = std::thread([this]{ loop(); });
    }

    void shutdown() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

private:
    void loop() {
        using namespace std::chrono;

        std::uint64_t expected_seq = 0;
        auto backoff = milliseconds(opt_.reconnectInitialMs);

        while (running_) {
            grpc::ClientContext ctx;
            auto stream = stub_->StreamFrames(&ctx); // bidirectional RW; client reads

            if (!stream) {
                std::cerr << "[grpc-client] connect failed\n";
                std::this_thread::sleep_for(backoff);
                backoff = std::min(backoff * 2, milliseconds(opt_.reconnectMaxMs));
                continue;
            }

            std::cout << "[grpc-client] connected to " << serverAddr_ << "\n";
            backoff = milliseconds(opt_.reconnectInitialMs);

            auto last_log = steady_clock::now();

            FrameChunk chunk;
            while (running_ && stream->Read(&chunk)) {
                if (chunk.width() == 0 || chunk.height() == 0 || chunk.data().empty()) {
                    if (opt_.printHeartbeat) {
                        auto now = steady_clock::now();
                        if (duration_cast<milliseconds>(now - last_log).count() >= opt_.idleLogMs) {
                            std::cout << "[grpc-client] heartbeat…\n";
                            last_log = now;
                        }
                    }
                    continue;
                }

                const auto calc_crc = crc32_bytes(chunk.data().data(), chunk.data().size());
                if (calc_crc != chunk.crc32()) {
                    std::cerr << "[grpc-client] CRC mismatch: seq=" << chunk.seq()
                              << " got=" << chunk.crc32()
                              << " calc=" << calc_crc << " → drop\n";
                    continue;
                }

                const auto seq = chunk.seq();
                if (expected_seq == 0) {
                    expected_seq = seq;
                } else if (seq > expected_seq + 1) {
                    std::cerr << "[grpc-client] gap: expected " << (expected_seq + 1)
                              << " got " << seq << " (lost " << (seq - expected_seq - 1) << ")\n";
                    expected_seq = seq;
                } else if (seq <= expected_seq) {
                    std::cerr << "[grpc-client] out-of-order or duplicate: seq=" << seq
                              << " expected " << (expected_seq + 1) << " → drop\n";
                    continue;
                } else {
                    expected_seq = seq;
                }

                Frame f{
                    std::span<const std::uint8_t>(
                        reinterpret_cast<const std::uint8_t*>(chunk.data().data()),
                        chunk.data().size()),
                    chunk.width(),
                    chunk.height(),
                    std::chrono::steady_clock::now(),
                    from_pb(chunk.format())
                };
                cb_(f);
            }

            if (!running_) break;
            std::cerr << "[grpc-client] stream closed → reconnecting…\n";
            std::this_thread::sleep_for(backoff);
            backoff = std::min(backoff * 2, milliseconds(opt_.reconnectMaxMs));
        }
    }

private:
    std::string serverAddr_;
    Options     opt_;

    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<SemStitch::Stub> stub_;

    std::atomic<bool> running_{false};
    std::thread worker_;
    FrameHandler cb_;
};

GrpcClient::GrpcClient(const std::string& serverAddr)
    : pimpl_{std::make_unique<Impl>(serverAddr, Options{})} {}

GrpcClient::GrpcClient(const std::string& serverAddr, Options opt)
    : pimpl_{std::make_unique<Impl>(serverAddr, opt)} {}

GrpcClient::~GrpcClient() = default;

void GrpcClient::start(FrameHandler cb) { pimpl_->start(std::move(cb)); }
void GrpcClient::shutdown()             { pimpl_->shutdown(); }

} // namespace semstitch
