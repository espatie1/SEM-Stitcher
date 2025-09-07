#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>

namespace semstitch {

struct ComposeOptions {
    bool   use_hann_weight      = true;   // attenuate tile edges with a Hann window
    bool   normalize_each_tile  = false;  // match tile brightness to the current mosaic
    double norm_clip_low        = 0.7;    // gain clamp
    double norm_clip_high       = 1.3;
    double eps                  = 1e-6;   // numerical stability
};

struct ComposeResult {
    cv::Mat  mosaic8u;   // final 8-bit mosaic (grayscale)
    cv::Size size;       // canvas size
    cv::Point2d origin;  // shift applied so all coordinates become >= 0
};

/** Compose a mosaic from tiles (Gray8) and their poses (absolute pixel coordinates). */
ComposeResult composeMosaic(const std::vector<cv::Mat>& tilesGray8,
                            const std::vector<cv::Point2d>& poses,
                            const ComposeOptions& opt = {});

} // namespace semstitch
