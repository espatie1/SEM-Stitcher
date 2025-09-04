#include "semstitch/core/Backend.hpp"
#include "cpu/CpuBackend.hpp"
#include <stdexcept>
#include <memory>

namespace semstitch {

std::unique_ptr<IBackend> makeBackend(BackendType type,
                                      std::size_t workerThreads /*=0*/)
{
    switch (type) {
        case BackendType::CPU:
        default:
            return std::make_unique<CpuBackend>(workerThreads);

        case BackendType::CUDA:
        case BackendType::OpenCL:
            throw std::runtime_error("Requested backend not yet implemented");
    }
}

} // namespace semstitch
