#include "semstitch/io/GrpcServer.hpp"
#include "semstitch.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <iostream>   

namespace semstitch {

class GrpcServer::Impl final : public SemStitch::Service {
public:
    explicit Impl(std::uint16_t port) {
        std::string addr = "0.0.0.0:" + std::to_string(port);
        grpc::ServerBuilder builder;
        builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
        builder.RegisterService(this);
        server_ = builder.BuildAndStart();
        std::cerr << "[grpc-server] listening at " << addr << std::endl; 
        worker_ = std::thread([this] { server_->Wait(); });
    }
    ~Impl() {
        server_->Shutdown();
        if (worker_.joinable()) worker_.join();
    }

    void push(const Frame& f) {
        std::lock_guard lk(mx_);
        q_.push(f);
        cv_.notify_all();
    }

private:
    grpc::Status StreamFrames(
        grpc::ServerContext* ctx,
        grpc::ServerReaderWriter<FrameChunk, FrameChunk>* stream) override {

        std::size_t sent = 0;
        while (!ctx->IsCancelled()) {
            std::unique_lock lk(mx_);
            cv_.wait(lk, [this, ctx] { return !q_.empty() || ctx->IsCancelled(); });
            while (!q_.empty()) {
                const Frame f = q_.front(); q_.pop();
                lk.unlock();

                FrameChunk out;
                out.set_data(reinterpret_cast<const char*>(f.data.data()),
                             static_cast<int>(f.bytes()));
                out.set_width(static_cast<int>(f.width));
                out.set_height(static_cast<int>(f.height));
                out.set_timestamp_ns(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        f.timestamp.time_since_epoch()).count());
                out.set_format(static_cast<PixelFmtPB>(f.format));

                if (!stream->Write(out)) {
                    std::cerr << "[grpc-server] stream->Write() failed, client gone?\n";
                    return grpc::Status::OK;
                }
                ++sent;
                if (sent % 30 == 0) {
                    std::cerr << "[grpc-server] sent frames: " << sent << std::endl;
                }
                lk.lock();
            }
        }
        return grpc::Status::OK;
    }

    std::unique_ptr<grpc::Server> server_;
    std::thread                   worker_;
    std::queue<Frame>             q_;
    std::mutex                    mx_;
    std::condition_variable       cv_;
};

GrpcServer::GrpcServer(std::uint16_t port)
    : pimpl_{std::make_unique<Impl>(port)} {}

GrpcServer::~GrpcServer() = default;
void GrpcServer::pushFrame(const Frame& f) { pimpl_->push(f); }

} // namespace semstitch
