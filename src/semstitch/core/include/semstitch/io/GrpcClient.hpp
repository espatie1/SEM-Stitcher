#pragma once

#include "semstitch/core/Frame.hpp"
#include <functional>
#include <memory>
#include <string>

namespace semstitch {

class GrpcClient final {
public:
    using FrameHandler = std::function<void(const Frame&)>;

    explicit GrpcClient(const std::string& address = "localhost:50051");
    void start(FrameHandler cb); 
    void shutdown();             

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace semstitch
