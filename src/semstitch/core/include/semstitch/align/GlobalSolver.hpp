#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "semstitch/align/GridAlign.hpp"

namespace semstitch {

/** Solver options. */
struct SolveOptions {
    int    max_iters        = 15;     // maximum IRLS iterations
    double huber_delta_px   = 3.0;    // Huber threshold (px) for vector residual (dx, dy)
    double anchor_weight    = 1e6;    // "stiffness" of the anchor for tile (0,0) at (0,0)
    double zncc_pow         = 2.0;    // boost good ZNCC: weight ~ (max(0, zncc))^zncc_pow
    double min_use_zncc     = 0.10;   // ignore edges with ZNCC below this
    double phase_resp_min   = 0.01;   // very weak lower threshold for phase correlation response
    double stop_eps         = 1e-4;   // stopping condition by relative ||delta||
};

/** Global solution result. */
struct SolveResult {
    std::vector<cv::Point2d> poses;   // (x,y) for tiles in row-major order: r*cols + c
    int    iters{0};
    double rmse_px{0.0};              // RMSE over all used edges
    int    used_edges{0};
    int    inliers{0};
    int    outliers{0};
    std::string report;               // short text log
};

/**
 * Global alignment by translations.
 * rows, cols — grid size; edges are taken from computePairMatches().
 * Returns positions (x, y) for all tiles, with tile (0,0) anchored at (0,0).
 */
SolveResult solveGlobalTranslation(const AlignResult& matches,
                                   int rows, int cols,
                                   const SolveOptions& opt = {});

/** Convenient initialization of poses using median steps (dx from "right", dy from "down"). */
std::vector<cv::Point2d> initializeByMedians(const AlignResult& matches,
                                             int rows, int cols);

} // namespace semstitch
