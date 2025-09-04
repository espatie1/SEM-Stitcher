#pragma once

#include "Frame.hpp"
#include "Config.hpp"
#include "DriftCorrector.hpp"

#include <opencv2/core.hpp>
#include <mutex>

namespace semstitch {

class Stitcher {
public:
    explicit Stitcher(IBackend& backend, const Config& cfg = {});

    void pushFrame(const Frame& frame);

    [[nodiscard]] cv::Mat snapshot() const;

    [[nodiscard]] std::size_t frameCount() const noexcept { return framesProcessed_; }

private:
    IBackend&      backend_;
    DriftCorrector drift_;
    Config         cfg_;

    cv::Mat mosaic_;
    cv::Mat lastFrame_;

    double offsetX_{0.0}, offsetY_{0.0};
    std::size_t framesProcessed_{0};
    bool first_{true};

    mutable std::mutex mtx_;
};

} // namespace semstitch
