#include "semstitch/io/GrpcServer.hpp"
#include "semstitch.grpc.pb.h"
#include <grpcpp/grpcpp.h>

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <atomic>

namespace semstitch {

struct BufferFrame {
    std::vector<std::uint8_t> data;
    std::uint32_t             w{0}, h{0};
    PixelFmtPB                fmt{PixelFmtPB::GRAY8};
    std::uint64_t             ts{0};
};

class GrpcServer::Impl final : public SemStitch::Service {
public:
    Impl(std::uint16_t port, Options opt) : opt_(opt) {
        const std::string addr = "0.0.0.0:" + std::to_string(port);
        grpc::ServerBuilder b;
        b.AddListeningPort(addr, grpc::InsecureServerCredentials());
        b.RegisterService(this);
        server_ = b.BuildAndStart();
        std::cout << "[grpc-server] listening at " << addr << "\n";

        if (opt_.heartbeatMs > 0) {
            hbRun_.store(true);
            hb_ = std::thread([this] {
                using namespace std::chrono;
                while (hbRun_.load()) {
                    std::this_thread::sleep_for(milliseconds(opt_.heartbeatMs));
                    if (!hbRun_.load()) break;
                    std::size_t qsz = 0;
                    {
                        std::lock_guard lk(mx_);
                        qsz = q_.size();
                    }
                    std::cout << "[grpc-server] queue=" << qsz << "\n";
                }
            });
        }
    }

    ~Impl() override {
        {
            std::lock_guard lk(mx_);
            stopped_ = true;
            cv_.notify_all();
        }
        if (server_) server_->Shutdown();

        hbRun_.store(false);
        if (hb_.joinable()) hb_.join();
    }

    void pushFrame(const Frame& f) {
        BufferFrame bf;
        bf.w   = f.width;
        bf.h   = f.height;
        bf.fmt = PixelFmtPB::GRAY8; // текущий симулятор отдаёт Gray8
        bf.ts  = std::chrono::duration_cast<std::chrono::nanoseconds>(
                   f.timestamp.time_since_epoch()).count();
        bf.data.assign(f.data.begin(), f.data.end());

        std::lock_guard lk(mx_);
        if ((int)q_.size() >= opt_.max_queue) {
            if (opt_.dropOldest) {
                if (!q_.empty()) q_.pop();
                q_.push(std::move(bf));
            } else {
                // просто игнорируем новый кадр
            }
        } else {
            q_.push(std::move(bf));
        }
        cv_.notify_one();
    }

private:
    // bi-di streaming
    grpc::Status StreamFrames(
        grpc::ServerContext* ctx,
        grpc::ServerReaderWriter<FrameChunk, FrameChunk>* stream) override
    {
        std::size_t sent = 0;
        while (true) {
            BufferFrame bf;
            {
                std::unique_lock lk(mx_);
                cv_.wait(lk, [this]{ return stopped_ || !q_.empty(); });
                if (stopped_ && q_.empty()) break;
                bf = std::move(q_.front());
                q_.pop();
            }

            FrameChunk out;
            out.set_width(bf.w);
            out.set_height(bf.h);
            out.set_timestamp_ns(bf.ts);
            out.set_format(bf.fmt);
            out.set_data(reinterpret_cast<const char*>(bf.data.data()), bf.data.size());

            if (!stream->Write(out)) break;
            ++sent;
            if (sent % 30 == 0) std::cout << "[grpc-server] streamed frames: " << sent << "\n";
            if (ctx->IsCancelled()) break;
        }
        return grpc::Status::OK;
    }

    Options                       opt_;
    std::unique_ptr<grpc::Server> server_;
    std::queue<BufferFrame>       q_;
    std::mutex                    mx_;
    std::condition_variable       cv_;
    bool                          stopped_{false};

    std::atomic<bool> hbRun_{false};
    std::thread       hb_;
};

GrpcServer::GrpcServer(std::uint16_t port)
    : pimpl_(std::make_unique<Impl>(port, Options{})) {}

GrpcServer::GrpcServer(std::uint16_t port, const Options& opt)
    : pimpl_(std::make_unique<Impl>(port, opt)) {}

GrpcServer::~GrpcServer() = default;

void GrpcServer::pushFrame(const Frame& f) { pimpl_->pushFrame(f); }

} // namespace semstitch
