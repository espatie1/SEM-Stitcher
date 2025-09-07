#include "semstitch/align/GridAlign.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace semstitch {

static cv::Rect safeRect(const cv::Size& sz, int x, int y, int w, int h) {
    x = std::max(0, x); y = std::max(0, y);
    w = std::min(w, sz.width  - x);
    h = std::min(h, sz.height - y);
    if (w < 1 || h < 1) return cv::Rect();
    return cv::Rect(x,y,w,h);
}

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

            // RIGHT neighbor
            if (c+1 < spec.cols) {
                const cv::Mat& B = tiles_gray8[idx(r,c+1)];
                CV_Assert(B.size()==A.size() && B.type()==CV_8UC1);

                cv::Rect roiA, roiB;
                pickOverlapROIs(A.size(), spec.overlap_percent, /*horizontal*/true, roiA, roiB);
                roiA = safeRect(A.size(), roiA.x, roiA.y, roiA.width, roiA.height);
                roiB = safeRect(B.size(), roiB.x, roiB.y, roiB.width, roiB.height);

                PairEdge e; e.r1=r; e.c1=c; e.r2=r; e.c2=c+1; e.direction="right";

                e.coarse = phaseCorrelateGray8(A, B, roiA, roiB, apod_win);

                // ограничение по механике (|dx|/|dy| не должны быть слишком большими)
                double bound = bound_factor * std::max(1.0, spec.stage_repeatability_px);
                bool inBound = (std::abs(e.coarse.dx) <= bound) && (std::abs(e.coarse.dy) <= bound);

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

            // DOWN neighbor
            if (r+1 < spec.rows) {
                const cv::Mat& B = tiles_gray8[idx(r+1,c)];
                CV_Assert(B.size()==A.size() && B.type()==CV_8UC1);

                cv::Rect roiA, roiB;
                pickOverlapROIs(A.size(), spec.overlap_percent, /*horizontal*/false, roiA, roiB);
                roiA = safeRect(A.size(), roiA.x, roiA.y, roiA.width, roiA.height);
                roiB = safeRect(B.size(), roiB.x, roiB.y, roiB.width, roiB.height);

                PairEdge e; e.r1=r; e.c1=c; e.r2=r+1; e.c2=c; e.direction="down";

                e.coarse = phaseCorrelateGray8(A, B, roiA, roiB, apod_win);

                double bound = bound_factor * std::max(1.0, spec.stage_repeatability_px);
                bool inBound = (std::abs(e.coarse.dx) <= bound) && (std::abs(e.coarse.dy) <= bound);

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
