#pragma once
#include <opencv2/core.hpp>
#include "semstitch/align/PhaseCorrelator.hpp"

namespace semstitch {

/*
  Local refinement (hill-climb) around 'init' within ±radius_px using ZNCC.
  Works on equal-size ROIs (both from Gray8) and returns a PairMatch with dx, dy,
  and score = ZNCC value.
  Requirement: roiA.size() == roiB.size().
*/
PairMatch refineLocalZNCC(const cv::Mat& a_gray8,
                          const cv::Mat& b_gray8,
                          const cv::Rect& roiA,
                          const cv::Rect& roiB,
                          cv::Point2d init,
                          int radius_px = 4,
                          int min_valid_side = 8);

} // namespace semstitch
