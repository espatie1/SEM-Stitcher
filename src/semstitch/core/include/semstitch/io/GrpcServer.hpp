#pragma once
#include "semstitch/core/Frame.hpp"
#include <cstdint>
#include <memory>

namespace semstitch {

class GrpcServer {
public:
    struct Options {
        int  max_queue        = 256;   // максимум кадров в очереди
        bool dropOldest       = true;  // при переполнении: true → выкидываем старые, false → игнорим новый
        int  heartbeatMs      = 1000;  // период логов состояния сервера (>0 — включить)
        bool enableCompression= false; // зарезервировано, пока не используется
    };

    explicit GrpcServer(std::uint16_t port = 50051);
    GrpcServer(std::uint16_t port, const Options& opt);
    ~GrpcServer();

    void pushFrame(const Frame& f);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
