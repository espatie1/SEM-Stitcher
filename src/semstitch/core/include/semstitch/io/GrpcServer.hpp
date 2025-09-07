#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <memory>

namespace semstitch {

class GrpcServer {
public:
    struct Options {
        int   maxQueue        = 256;   // предел очереди кадров
        bool  dropOldest      = true;  // true: выталкивать старые; false: отбрасывать новые
        int   heartbeatMs     = 1000;  // период heartbeats, если нет данных
        bool  enableCompression = false; // зарезервировано (grpc-сжатие здесь не включаем)
    };

    explicit GrpcServer(std::uint16_t port = 50051);            // конструктор по умолчанию
    GrpcServer(std::uint16_t port, Options opt);                // с заданными опциями
    ~GrpcServer();

    void pushFrame(const Frame& f);

    GrpcServer(const GrpcServer&)            = delete;
    GrpcServer& operator=(const GrpcServer&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace semstitch
