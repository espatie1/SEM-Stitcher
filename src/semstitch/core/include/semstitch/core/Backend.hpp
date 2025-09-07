#pragma once

#include <memory>
#include "Frame.hpp"
#include "Config.hpp"

namespace semstitch {

/* Simple translation vector between two frames. */
struct DriftVector { double dx{0.0}, dy{0.0}; };

/* 3x3 homography matrix stored in row-major order. */
struct Homography  { double h[9] {1,0,0, 0,1,0, 0,0,1}; };

/*
  Backend interface for motion estimation.

  Implementations should provide:
    - drift(): estimate pure translation between two frames.
    - match(): estimate a homography (here can be translation-only or full 3x3).
*/
class IBackend {
public:
    virtual ~IBackend() = default;

    [[nodiscard]] virtual DriftVector drift(const Frame& prev, const Frame& curr) = 0;

    [[nodiscard]] virtual Homography  match(const Frame& prev, const Frame& curr) = 0;
};

/* Factory function for creating a backend of the requested type. */
std::unique_ptr<IBackend> makeBackend(BackendType type, std::size_t workerThreads = 0);

} // namespace semstitch
