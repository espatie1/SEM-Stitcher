#pragma once
#include <opencv2/core.hpp>

namespace semstitch {

/** Pairwise match result. */
struct PairMatch {
    double dx{0.0};      // shift of B relative to A (A(x,y) ~ B(x - dx, y - dy))
    double dy{0.0};
    double score{0.0};   // confidence (response from phaseCorrelate)
    bool   ok{false};
};

/*
  Phase correlation on overlapping ROIs (8-bit grayscale images).
  roiA and roiB must have the same size.
  If hanning_win > 0, apply a 2D Hanning window (edge apodization).
*/
PairMatch phaseCorrelateGray8(const cv::Mat& a_gray8,
                              const cv::Mat& b_gray8,
                              const cv::Rect& roiA,
                              const cv::Rect& roiB,
                              int hanning_win = 16);

} // namespace semstitch
