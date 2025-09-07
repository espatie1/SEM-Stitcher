#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <memory>

namespace semstitch {

class GrpcServer {
public:
    struct Options {
        int   maxQueue          = 256;   // maximum size of the frame queue
        bool  dropOldest        = true;  // true: evict oldest; false: drop newest
        int   heartbeatMs       = 1000;  // heartbeat period when no data (ms)
        bool  enableCompression = false; // reserved (do not enable gRPC compression here)
    };

    explicit GrpcServer(std::uint16_t port = 50051); // constructor with default options
    GrpcServer(std::uint16_t port, Options opt);     // constructor with custom options
    ~GrpcServer();

    void pushFrame(const Frame& f);

    GrpcServer(const GrpcServer&)            = delete;
    GrpcServer& operator=(const GrpcServer&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace semstitch
