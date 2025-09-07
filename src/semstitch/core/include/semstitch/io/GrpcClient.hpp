#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace semstitch {

class GrpcClient {
public:
    struct Options {
        int   reconnectInitialMs = 300;   // initial reconnect backoff (ms)
        int   reconnectMaxMs     = 8000;  // maximum reconnect backoff (ms)
        int   idleLogMs          = 2500;  // heartbeat log period when idle (ms)
        bool  enableCompression  = false; // reserved
        bool  printHeartbeat     = true;  // print periodic heartbeat logs
    };

    using FrameHandler = std::function<void(const Frame&)>;

    explicit GrpcClient(const std::string& serverAddr);     // constructor with default options
    GrpcClient(const std::string& serverAddr, Options opt); // constructor with custom options
    ~GrpcClient();

    void start(FrameHandler cb);
    void shutdown();

    GrpcClient(const GrpcClient&)            = delete;
    GrpcClient& operator=(const GrpcClient&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
