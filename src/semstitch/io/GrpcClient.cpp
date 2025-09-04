#include "semstitch/io/GrpcClient.hpp"
#include "semstitch.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <thread>

namespace semstitch {

class GrpcClient::Impl {
public:
    explicit Impl(const std::string& addr)
        : stub_{SemStitch::NewStub(
              grpc::CreateChannel(addr,
                                  grpc::InsecureChannelCredentials()))} {}

    void start(FrameHandler cb) {
        running_ = true;
        worker_  = std::thread([this, cb] { run(cb); });
    }

    void shutdown() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

private:
    void run(FrameHandler cb) {
        grpc::ClientContext ctx;
        auto stream = stub_->StreamFrames(&ctx);
        FrameChunk chunk;

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
        }
    }

    std::unique_ptr<SemStitch::Stub> stub_;
    bool        running_{false};
    std::thread worker_;
};

GrpcClient::GrpcClient(const std::string& addr)
    : pimpl_{std::make_unique<Impl>(addr)} {}

GrpcClient::~GrpcClient() = default; 

void GrpcClient::start(FrameHandler cb) { pimpl_->start(std::move(cb)); }
void GrpcClient::shutdown()             { pimpl_->shutdown(); }

} // namespace semstitch
