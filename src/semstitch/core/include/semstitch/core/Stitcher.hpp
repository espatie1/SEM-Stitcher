#pragma once
#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Frame.hpp"
#include "semstitch/compose/Mosaic.hpp"

#include <opencv2/core.hpp>
#include <mutex>
#include <vector>

namespace semstitch {

/// Incremental stitcher:
/// - accepts frames (Gray8);
/// - estimates the shift of `curr` relative to `prev` (phaseCorrelate);
/// - snapshot() calls composeMosaic() (Hann weights + normalization).
class Stitcher {
public:
    struct Options {
        // limit the history size (avoid growing memory)
        int  maxTiles         = 2000;

        // shift estimation parameters
        bool useApodForPhase  = true; // Hann window before phaseCorrelate

        // compositor options
        ComposeOptions compose{
            /*use_hann_weight=*/true,
            /*normalize_each_tile=*/true,
            /*norm_clip_low=*/0.8,
            /*norm_clip_high=*/1.25,
            /*eps=*/1e-6f
        };
    };

    // IMPORTANT: no default argument here — otherwise GCC complains
    explicit Stitcher(IBackend& backend);
    Stitcher(IBackend& backend, const Options& opt);

    /// add a frame to the stream
    void pushFrame(const Frame& f);

    /// get the current mosaic (8-bit, single channel)
    cv::Mat snapshot() const;

    /// change compose options on the fly
    void setComposeOptions(const ComposeOptions& co);

    Options options() const { return opt_; }

private:
    IBackend& backend_;
    Options   opt_;

    // accumulated tiles and their poses (in mosaic coordinates; double = subpixel)
    std::vector<cv::Mat>     tiles_;   // CV_8UC1
    std::vector<cv::Point2d> poses_;   // top-left corner

    // for incremental estimation
    cv::Mat      lastTile_;            // CV_8UC1
    cv::Point2d  lastPose_{0.0, 0.0};
    bool         haveLast_{false};

    mutable std::mutex mtx_;

    // estimate relative shift of curr w.r.t. prev (prev -> curr)
    cv::Point2d estimateOffset(const cv::Mat& prev8u, const cv::Mat& curr8u) const;

    // trim history if it exceeds the limit
    void enforceHistoryLimitLocked();
};

} // namespace semstitch
