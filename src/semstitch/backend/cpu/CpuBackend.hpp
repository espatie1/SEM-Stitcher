#pragma once

#include "semstitch/core/Backend.hpp"

namespace semstitch {

class CpuBackend final : public IBackend {
public:
    explicit CpuBackend(std::size_t /*threads*/ = 0) {}

    DriftVector drift(const Frame& prev, const Frame& curr) override;
    Homography  match(const Frame& prev, const Frame& curr) override;
};

} // namespace semstitch

