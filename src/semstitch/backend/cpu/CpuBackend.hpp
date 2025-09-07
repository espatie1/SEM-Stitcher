#pragma once

#include "semstitch/core/Backend.hpp"

namespace semstitch {

/*
  CPU-based backend implementation.

  This class estimates:
    - frame-to-frame drift (translation) using phase correlation,
    - a simple homography that currently encodes only translation.

  Notes:
    - Input frames are expected to be 8-bit grayscale.
    - The constructor parameter 'threads' is not used yet.
*/
class CpuBackend final : public IBackend {
public:
    // Construct a CPU backend. 'threads' is reserved for future use.
    explicit CpuBackend(std::size_t /*threads*/ = 0) {}

    // Estimate subpixel translation between two frames.
    DriftVector drift(const Frame& prev, const Frame& curr) override;

    // Build a homography between two frames (currently translation only).
    Homography  match(const Frame& prev, const Frame& curr) override;
};

} // namespace semstitch
