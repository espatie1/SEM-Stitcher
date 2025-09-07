#pragma once
#include <opencv2/core.hpp>

namespace semstitch {

/** Итог парного матчинга. */
struct PairMatch {
    double dx{0.0};      // сдвиг B относительно A (A(x,y) ~ B(x-dx, y-dy))
    double dy{0.0};
    double score{0.0};   // "доверие" (response от phaseCorrelate)
    bool   ok{false};
};

/**
 * Фазовая корреляция на наложенных ROI (серые 8-бит кадры).
 * roiA и roiB должны быть одинакового размера.
 * hanning_win>0 применяет 2D Hanning-окно (аподизация краёв).
 */
PairMatch phaseCorrelateGray8(const cv::Mat& a_gray8,
                              const cv::Mat& b_gray8,
                              const cv::Rect& roiA,
                              const cv::Rect& roiB,
                              int hanning_win = 16);

} // namespace semstitch
