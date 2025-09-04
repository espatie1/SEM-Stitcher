#pragma once


#include <cstddef>
#include <cstdint>


namespace semstitch {


enum class BackendType : std::uint8_t {
    CPU = 0,
    CUDA = 1,
    OpenCL = 2
};


struct Config {
    std::size_t pyramidLevels {4};
    bool enableKalman {true};

    BackendType backend {BackendType::CPU};
    std::size_t workerThreads {0}; 

    bool gpuAsyncCopy {true};
    std::size_t ringBufferSize {16};
    std::size_t maxMosaicTiles {4096};
};


} // namespace semstitch