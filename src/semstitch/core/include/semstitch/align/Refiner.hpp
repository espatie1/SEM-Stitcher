#pragma once
#include <opencv2/core.hpp>
#include "semstitch/align/PhaseCorrelator.hpp"

namespace semstitch {

/**
 * Локальная донастройка (hill-climb) в окрестности init (±radius_px), используя ZNCC.
 * Работает на одинаковых ROI (оба из Gray8), возвращает PairMatch с dx,dy и score=ZNCC.
 * Требует: roiA.size()==roiB.size().
 */
PairMatch refineLocalZNCC(const cv::Mat& a_gray8,
                          const cv::Mat& b_gray8,
                          const cv::Rect& roiA,
                          const cv::Rect& roiB,
                          cv::Point2d init,
                          int radius_px = 4,
                          int min_valid_side = 8);

} // namespace semstitch
