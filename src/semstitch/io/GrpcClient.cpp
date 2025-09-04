#include "semstitch/io/GrpcClient.hpp"
#include "semstitch.grpc.pb.h"
#include <grpcpp/grpcpp.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

namespace semstitch {

class GrpcClient::Impl {
public:
    Impl(std::string addr, Options opt) : addr_(std::move(addr)), opt_(opt) {}
    ~Impl() { shutdown(); }

    void start(FrameHandler cb) {
        if (running_.exchange(true)) return;
        reader_ = std::thread([this, cb]{ loop(cb); });
        if (opt_.print_heartbeat)
            hb_ = std::thread([this]{ heartbeat(); });
    }

    void shutdown() {
        running_ = false;
        if (reader_.joinable()) reader_.join();
        if (hb_.joinable())     hb_.join();
    }

private:
    void loop(FrameHandler cb) {
        auto channel = grpc::CreateChannel(addr_, grpc::InsecureChannelCredentials());
        auto stub    = SemStitch::NewStub(channel);

        std::cout << "[grpc-client] connected to " << addr_ << "\n";

        using namespace std::chrono;
        int backoff = opt_.reconnectInitialMs;

        while (running_) {
            grpc::ClientContext ctx;
            FrameChunk chunk;
            auto stream = stub->StreamFrames(&ctx); // bi-di: reader-writer

            std::size_t n = 0;
            while (running_ && stream->Read(&chunk)) {
                Frame f{
                    std::span<const std::uint8_t>(
                        reinterpret_cast<const std::uint8_t*>(chunk.data().data()),
                        chunk.data().size()),
                    chunk.width(),
                    chunk.height(),
                    std::chrono::steady_clock::now(),
                    static_cast<PixelFormat>(chunk.format())
                };
                cb(f);
                ++n;
                lastCount_.store(lastCount_.load() + 1, std::memory_order_relaxed);
                backoff = opt_.reconnectInitialMs; // успешное чтение — сбрасываем бэкофф
            }

            if (!running_) break;

            // Стрим разорвался — подождём и переподключимся.
            std::this_thread::sleep_for(milliseconds(backoff));
            backoff = std::min(backoff * 2, opt_.reconnectMaxMs);
        }

        running_ = false;
    }

    void heartbeat() {
        using namespace std::chrono;
        std::size_t prev = 0;
        while (running_) {
            std::this_thread::sleep_for(milliseconds(opt_.idleLogMs));
            auto cur = lastCount_.load(std::memory_order_relaxed);
            if (cur == prev) std::cout << "[grpc-client] heartbeat…\n";
            prev = cur;
        }
    }

    std::string              addr_;
    Options                  opt_;
    std::atomic<bool>        running_{false};
    std::atomic<std::size_t> lastCount_{0};
    std::thread              reader_, hb_;
};

GrpcClient::GrpcClient(const std::string& address)
    : pimpl_(std::make_unique<Impl>(address, Options{})) {}

GrpcClient::GrpcClient(const std::string& address, const Options& opt)
    : pimpl_(std::make_unique<Impl>(address, opt)) {}

GrpcClient::~GrpcClient() = default;
void GrpcClient::start(FrameHandler cb) { pimpl_->start(std::move(cb)); }
void GrpcClient::shutdown()             { pimpl_->shutdown(); }

} // namespace semstitch
