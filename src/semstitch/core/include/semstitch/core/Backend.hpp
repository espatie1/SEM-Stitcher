#pragma once

#include <memory>
#include "Frame.hpp"
#include "Config.hpp"

namespace semstitch {

struct DriftVector { double dx{0.0}, dy{0.0}; };
struct Homography  { double h[9] {1,0,0, 0,1,0, 0,0,1}; };

class IBackend {
public:
    virtual ~IBackend() = default;

    [[nodiscard]] virtual DriftVector drift(const Frame& prev, const Frame& curr) = 0;

    [[nodiscard]] virtual Homography  match(const Frame& prev, const Frame& curr) = 0;
};

std::unique_ptr<IBackend> makeBackend(BackendType type, std::size_t workerThreads = 0);

} // namespace semstitch
