#pragma once

#include "Backend.hpp"

namespace semstitch {

class DriftCorrector {
public:
    explicit DriftCorrector(IBackend& backend) noexcept : backend_{backend} {}

    [[nodiscard]] DriftVector operator()(const Frame& prev, const Frame& curr) {
        return backend_.drift(prev, curr);
    }

private:
    IBackend& backend_;
};

}

