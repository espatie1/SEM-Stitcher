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
        int   reconnectInitialMs = 300;   // начальный бэкофф при реконнекте
        int   reconnectMaxMs     = 8000;  // верхняя граница бэкоффа
        int   idleLogMs          = 2500;  // период heartbeat-лога в простое
        bool  enableCompression  = false; // зарезервировано
        bool  printHeartbeat     = true;
    };

    using FrameHandler = std::function<void(const Frame&)>;

    explicit GrpcClient(const std::string& serverAddr);     // конструктор по умолчанию
    GrpcClient(const std::string& serverAddr, Options opt); // с заданными опциями
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
