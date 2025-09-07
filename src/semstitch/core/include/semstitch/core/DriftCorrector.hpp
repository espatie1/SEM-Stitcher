#pragma once

#include "Backend.hpp"

namespace semstitch {

/*
  Thin wrapper that delegates drift estimation to a backend.
  Usage:
    DriftCorrector dc(backend);
    DriftVector d = dc(prev, curr);
*/
class DriftCorrector {
public:
    explicit DriftCorrector(IBackend& backend) noexcept : backend_{backend} {}

    // Call operator: estimate translation (dx, dy) between frames.
    [[nodiscard]] DriftVector operator()(const Frame& prev, const Frame& curr) {
        return backend_.drift(prev, curr);
    }

private:
    IBackend& backend_;
};

} // namespace semstitch
