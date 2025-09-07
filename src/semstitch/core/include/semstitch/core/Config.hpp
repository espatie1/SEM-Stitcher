#pragma once

#include <cstddef>
#include <cstdint>

namespace semstitch {

/* Backend implementation types. */
enum class BackendType : std::uint8_t {
    CPU = 0,
    CUDA = 1,
    OpenCL = 2
};

/* Global configuration for the stitching pipeline.
   This struct can be expanded and passed across modules. */
struct Config {
    std::size_t pyramidLevels {4};   // number of image pyramid levels
    bool enableKalman {true};        // enable simple Kalman smoothing

    BackendType backend {BackendType::CPU}; // which backend to use
    std::size_t workerThreads {0};          // 0 = auto / use default

    bool gpuAsyncCopy {true};        // allow async GPU copies (if supported)
    std::size_t ringBufferSize {16}; // number of buffered frames
    std::size_t maxMosaicTiles {4096}; // safety limit for total tiles
};

} // namespace semstitch
