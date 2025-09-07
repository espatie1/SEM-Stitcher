#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "semstitch/align/PhaseCorrelator.hpp"
#include "semstitch/align/Refiner.hpp"

namespace semstitch {

/** Описание сетки. */
struct GridSpec {
    int rows{0};
    int cols{0};
    double overlap_percent{10.0};        // % перекрытия по каждой оси
    double stage_repeatability_px{3.0};   // повторяемость стола (px)
};

/** Рёбра матчинга между соседями. */
struct PairEdge {
    int r1{0}, c1{0};
    int r2{0}, c2{0};
    PairMatch coarse;  // phase corr
    PairMatch fine;    // ZNCC refine
    bool      ok{false};
    std::string direction; // "right" или "down"
};

struct AlignResult {
    std::vector<PairEdge> edges; // только соседние r,c → r,c+1 и r,c → r+1,c
};

/** Построить парные соответствия по сетке (только translation). */
AlignResult computePairMatches(const std::vector<cv::Mat>& tiles_gray8,
                               const GridSpec& spec,
                               int apod_win = 16,
                               int refine_radius = 4,
                               double phase_resp_thresh = 0.02,
                               double zncc_thresh = 0.20,
                               double bound_factor = 4.0);

} // namespace semstitch
