#pragma once
#include "semstitch/core/Frame.hpp"
#include <memory>
#include <cstdint>

namespace grpc { class Server; }

namespace semstitch {

class GrpcServer final {
public:
    struct Options {
        std::size_t maxQueue = 128;   // максимальная длина очереди кадров
        bool dropOldest = true;       // true → выкидываем самый старый, false → отбрасываем новый
        int heartbeatMs = 1000;       // если нет кадров столько мс — шлём heartbeat (пустой кадр)
        bool enableCompression = false;
    };

    explicit GrpcServer(std::uint16_t port = 50051);
    GrpcServer(std::uint16_t port, Options opts);
    ~GrpcServer();

    void pushFrame(const Frame& f);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
