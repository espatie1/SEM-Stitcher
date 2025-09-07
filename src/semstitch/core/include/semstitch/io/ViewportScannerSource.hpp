#pragma once
#include "semstitch/core/Frame.hpp"

#include <opencv2/core.hpp>
#include <optional>
#include <random>
#include <chrono>
#include <string>
#include <vector>

namespace semstitch {

/**
 * Frame source that “scans” a viewport (tile) over a master image
 * (either loaded from disk or generated as a synthetic pattern).
 *
 * Supports: zig-zag grid traversal, overlap, drift, jitter, flicker.
 */
class ViewportScannerSource {
public:
    struct Options {
        int    tileW = 512, tileH = 512;       // tile size (pixels)
        int    gridCols = 5, gridRows = 5;     // scan grid
        double overlap = 0.20;                 // overlap 0..1
        double driftX = 5.0, driftY = 2.5;     // drift (pixels per second)
        double jitterSigma = 0.4;              // RMS jitter (pixels)
        double flickerAmp  = 0.03;             // brightness flicker amplitude (0..1)
        int    masterW = 4096, masterH = 4096; // synthetic scene size if no file
        unsigned seed = 0;                     // RNG seed (0 = auto)

        // New:
        std::string masterPath{};              // if set, load this file as the master
        std::string pattern = "grid";          // "grid", "rings", "text:SEM", "checker", "noise"
    };

    // IMPORTANT: no default argument here — to avoid a GCC12 issue
    explicit ViewportScannerSource(const Options& opt);
    // Delegating default constructor
    ViewportScannerSource();

    std::optional<Frame> next();               // next frame

private:
    Options opt_;
    cv::Mat master_;                           // 8UC1
    std::vector<std::uint8_t> scratch_;        // return buffer

    int curIdx_ = 0;                           // current tile index
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};
    std::chrono::steady_clock::time_point t0_;

    void ensureMasterReady();
    cv::Mat makePattern(int w, int h, const std::string& pat); // 8UC1
};

} // namespace semstitch
