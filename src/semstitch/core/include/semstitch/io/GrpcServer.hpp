#pragma once

#include "semstitch/core/Frame.hpp"
#include <memory>
#include <cstdint>

namespace grpc { class Server; }

namespace semstitch {

class GrpcServer final {
public:
    explicit GrpcServer(std::uint16_t port = 50051);
    ~GrpcServer();

    void pushFrame(const Frame& f);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
