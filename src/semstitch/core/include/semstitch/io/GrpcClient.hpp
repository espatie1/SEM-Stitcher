#pragma once
#include "semstitch/core/Frame.hpp"
#include <functional>
#include <memory>
#include <string>

namespace semstitch {

class GrpcClient {
public:
    using FrameHandler = std::function<void(const Frame&)>;

    struct Options {
        bool print_heartbeat   = true;  // печатать пульсацию
        int  idleLogMs         = 2000;  // период heartbeat
        int  reconnectInitialMs= 250;   // начальная задержка перед переподключением
        int  reconnectMaxMs    = 5000;  // максимум задержки
        bool enableCompression = false; // зарезервировано, пока не используется
    };

    explicit GrpcClient(const std::string& address = "localhost:50051");
    GrpcClient(const std::string& address, const Options& opt);
    ~GrpcClient();

    void start(FrameHandler cb);
    void shutdown();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
