#include "semstitch/align/GridAlign.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

/*
  Build pairwise matches between neighboring tiles in a grid.

  For each tile, we compare it with:
    - the RIGHT neighbor (same row, next column),
    - the DOWN neighbor (next row, same column).

  We take only the overlap area between tiles.
  First we run phase correlation (coarse dx, dy and a response score).
  Then we refine with local ZNCC around the coarse shift.
  We keep an edge if scores pass thresholds and the shift is within a
  mechanical bound (stage repeatability).

  Output is an AlignResult that holds a list of PairEdge records.
*/

namespace semstitch {

/* Clamp a rectangle to image bounds.
   If the rectangle is empty after clamping, return an empty Rect. */
static cv::Rect safeRect(const cv::Size& sz, int x, int y, int w, int h) {
    x = std::max(0, x); y = std::max(0, y);
    w = std::min(w, sz.width  - x);
    h = std::min(h, sz.height - y);
    if (w < 1 || h < 1) return cv::Rect();
    return cv::Rect(x,y,w,h);
}

/* Pick ROIs that cover the expected overlap area between two tiles.
   If horizontal == true:
     - roiA is the right strip of A, roiB is the left strip of B.
   Else (vertical):
     - roiA is the bottom strip of A, roiB is the top strip of B.

   overlap_percent is given in percent of width/height.
   We also enforce a small minimum (8 px) so the strip is not too thin. */
static void pickOverlapROIs(const cv::Size& sz,
                            double overlap_percent,
                            bool horizontal,
                            cv::Rect& roiA, cv::Rect& roiB)
{
    int W = sz.width, H = sz.height;
    int ovw = std::max(8, int(std::round(W * (overlap_percent * 0.01))));
    int ovh = std::max(8, int(std::round(H * (overlap_percent * 0.01))));
    if (horizontal) {
        roiA = cv::Rect(W-ovw, 0, ovw, H);
        roiB = cv::Rect(0,     0, ovw, H);
    } else {
        roiA = cv::Rect(0, H-ovh, W, ovh);
        roiB = cv::Rect(0, 0,     W, ovh);
    }
}

/*
  Compute pair matches for all neighbor pairs in a grid.

  tiles_gray8   : vector of tiles (grayscale, CV_8UC1), size must be rows*cols.
  spec          : grid specification (rows, cols, expected overlap, etc.).
  apod_win      : apodization window radius (pixels) for phase correlation.
  refine_radius : ZNCC local search radius around coarse (dx, dy).
  phase_resp_thresh : minimal phase correlation response to accept coarse match.
  zncc_thresh        : minimal ZNCC score to accept refined match.
  bound_factor       : limit on |dx| and |dy|, scaled by stage_repeatability_px.
                       (prevents unrealistic large shifts)

  Return: AlignResult with all PairEdge records (right/down edges).
*/
AlignResult computePairMatches(const std::vector<cv::Mat>& tiles_gray8,
                               const GridSpec& spec,
                               int apod_win,
                               int refine_radius,
                               double phase_resp_thresh,
                               double zncc_thresh,
                               double bound_factor)
{
    AlignResult out;
    if (spec.rows<=0 || spec.cols<=0) return out;
    CV_Assert(int(tiles_gray8.size()) == spec.rows*spec.cols);

    auto idx = [&](int r, int c){ return r*spec.cols + c; };

    for (int r=0; r<spec.rows; ++r) {
        for (int c=0; c<spec.cols; ++c) {
            const cv::Mat& A = tiles_gray8[idx(r,c)];
            CV_Assert(!A.empty() && A.type()==CV_8UC1);

            // RIGHT neighbor: compare A(r,c) with B(r,c+1)
            if (c+1 < spec.cols) {
                const cv::Mat& B = tiles_gray8[idx(r,c+1)];
                CV_Assert(B.size()==A.size() && B.type()==CV_8UC1);

                cv::Rect roiA, roiB;
                pickOverlapROIs(A.size(), spec.overlap_percent, /*horizontal*/true, roiA, roiB);
                roiA = safeRect(A.size(), roiA.x, roiA.y, roiA.width, roiA.height);
                roiB = safeRect(B.size(), roiB.x, roiB.y, roiB.width, roiB.height);

                PairEdge e; e.r1=r; e.c1=c; e.r2=r; e.c2=c+1; e.direction="right";

                // Coarse shift by phase correlation (also gives a response score)
                e.coarse = phaseCorrelateGray8(A, B, roiA, roiB, apod_win);

                // Mechanical bound: |dx| and |dy| should not be too large
                double bound = bound_factor * std::max(1.0, spec.stage_repeatability_px);
                bool inBound = (std::abs(e.coarse.dx) <= bound) && (std::abs(e.coarse.dy) <= bound);

                // If coarse is valid and inside bounds, refine with local ZNCC
                if (e.coarse.ok && e.coarse.score >= phase_resp_thresh && inBound) {
                    e.fine = refineLocalZNCC(A, B, roiA, roiB,
                                             cv::Point2d(e.coarse.dx, e.coarse.dy),
                                             refine_radius);
                    e.ok = e.fine.ok && (e.fine.score >= zncc_thresh);
                } else {
                    e.ok = false;
                }
                out.edges.push_back(e);
            }

            // DOWN neighbor: compare A(r,c) with B(r+1,c)
            if (r+1 < spec.rows) {
                const cv::Mat& B = tiles_gray8[idx(r+1,c)];
                CV_Assert(B.size()==A.size() && B.type()==CV_8UC1);

                cv::Rect roiA, roiB;
                pickOverlapROIs(A.size(), spec.overlap_percent, /*horizontal*/false, roiA, roiB);
                roiA = safeRect(A.size(), roiA.x, roiA.y, roiA.width, roiA.height);
                roiB = safeRect(B.size(), roiB.x, roiB.y, roiB.width, roiB.height);

                PairEdge e; e.r1=r; e.c1=c; e.r2=r+1; e.c2=c; e.direction="down";

                // Coarse shift by phase correlation (also gives a response score)
                e.coarse = phaseCorrelateGray8(A, B, roiA, roiB, apod_win);

                // Same mechanical bound check for (dx, dy)
                double bound = bound_factor * std::max(1.0, spec.stage_repeatability_px);
                bool inBound = (std::abs(e.coarse.dx) <= bound) && (std::abs(e.coarse.dy) <= bound);

                // Refine only if coarse is good and inside bounds
                if (e.coarse.ok && e.coarse.score >= phase_resp_thresh && inBound) {
                    e.fine = refineLocalZNCC(A, B, roiA, roiB,
                                             cv::Point2d(e.coarse.dx, e.coarse.dy),
                                             refine_radius);
                    e.ok = e.fine.ok && (e.fine.score >= zncc_thresh);
                } else {
                    e.ok = false;
                }
                out.edges.push_back(e);
            }
        }
    }
    return out;
}

} // namespace semstitch
