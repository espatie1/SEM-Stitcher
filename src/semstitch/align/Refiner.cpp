#include "semstitch/align/Refiner.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace semstitch {

/* Check that ROI r is fully inside image m and has positive size. */
static bool validROI(const cv::Mat& m, const cv::Rect& r) {
    return r.x>=0 && r.y>=0 && r.x+r.width<=m.cols && r.y+r.height<=m.rows && r.width>0 && r.height>0;
}

/* ZNCC(A, B) on the common area when B is shifted by integer (dx, dy) relative to A. */
static bool zncc_at_shift(const cv::Mat& A8, const cv::Mat& B8,
                          const cv::Rect& roiA, const cv::Rect& roiB,
                          int dx, int dy, double& out)
{
    CV_Assert(A8.type()==CV_8UC1 && B8.type()==CV_8UC1);
    // use local windows with the same size
    const int W = roiA.width;
    const int H = roiA.height;

    const int w = W - std::abs(dx);
    const int h = H - std::abs(dy);
    if (w < 8 || h < 8) return false;

    cv::Rect aR = roiA, bR = roiB;
    if (dx >= 0) { aR.x += dx; bR.x += 0;  } else { aR.x += 0;  bR.x += -dx; }
    if (dy >= 0) { aR.y += dy; bR.y += 0;  } else { aR.y += 0;  bR.y += -dy; }
    aR.width = bR.width = w;
    aR.height = bR.height = h;

    if (!validROI(A8,aR) || !validROI(B8,bR)) return false;

    cv::Mat A = A8(aR), B = B8(bR);
    cv::Scalar meanA, stdA, meanB, stdB;
    cv::meanStdDev(A, meanA, stdA);
    cv::meanStdDev(B, meanB, stdB);

    double sA = stdA[0], sB = stdB[0];
    // if one window is almost constant, treat ZNCC as invalid (very low)
    if (sA < 1e-6 || sB < 1e-6) { out = -1.0; return true; }

    cv::Mat Af, Bf;
    A.convertTo(Af, CV_32F);
    B.convertTo(Bf, CV_32F);
    Af = Af - float(meanA[0]);
    Bf = Bf - float(meanB[0]);

    double num = cv::sum(Af.mul(Bf))[0];
    double den = (double(A.total()) - 1.0) * sA * sB;
    out = num / den;           // ZNCC in [-1..1]
    return true;
}

/* Subpixel correction by classic 1D parabolic interpolation.
   Input values are f(-1), f(0), f(+1) along an axis.
   Return is a small offset in [-1..1], usually about [-0.5..0.5]. */
static double subpixel_offset(double fm1, double f0, double fp1) {
    double denom = (fm1 - 2.0*f0 + fp1);
    if (std::abs(denom) < 1e-12) return 0.0;
    double delta = 0.5 * (fm1 - fp1) / denom; // typically in [-0.5; 0.5] for smooth peaks
    if (!std::isfinite(delta)) return 0.0;
    if (delta < -1.0) delta = -1.0;
    if (delta >  1.0) delta =  1.0;
    return delta;
}

/* Local ZNCC-based refinement around an initial integer shift.

   Inputs:
     - a_gray8, b_gray8 : grayscale tiles (CV_8UC1).
     - roiA, roiB       : same-size ROIs in A and B.
     - init             : initial shift (dx, dy) from a coarse method.
     - radius_px        : search radius (in pixels) around init for integer search.
     - min_valid_side   : (unused here) minimal side of the common window.

   Method:
     1) Round init to integers (cx, cy).
     2) Do a small brute-force search in the square [-r..r] around (cx, cy).
        Keep the best ZNCC score.
     3) Apply subpixel 1D parabola fit along X (fix Y), then along Y (fix X).
     4) Output refined (dx, dy) and the best ZNCC score.

   Returns PairMatch with:
     - dx, dy   : refined shift (can be subpixel),
     - score    : best ZNCC value in [-1..1],
     - ok       : true if dx, dy, and score are finite.
*/
PairMatch refineLocalZNCC(const cv::Mat& a_gray8,
                          const cv::Mat& b_gray8,
                          const cv::Rect& roiA,
                          const cv::Rect& roiB,
                          cv::Point2d init,
                          int radius_px,
                          int /*min_valid_side*/)
{
    CV_Assert(a_gray8.type()==CV_8UC1 && b_gray8.type()==CV_8UC1);
    CV_Assert(roiA.size() == roiB.size());

    // start from the nearest integer position
    int cx = int(std::round(init.x));
    int cy = int(std::round(init.y));

    double best = -2.0;
    int bestx = cx, besty = cy;

    // hill-climb in a small neighborhood (simple brute force in a small radius)
    for (int dy = -radius_px; dy <= radius_px; ++dy) {
        for (int dx = -radius_px; dx <= radius_px; ++dx) {
            double s = -2.0;
            if (!zncc_at_shift(a_gray8, b_gray8, roiA, roiB, cx+dx, cy+dy, s)) continue;
            if (s > best) { best = s; bestx = cx+dx; besty = cy+dy; }
        }
    }

    // subpixel correction along X (keep Y fixed)
    double fxm1=-2, fx0=-2, fxp1=-2;
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx-1, besty, fxm1);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx,   besty, fx0);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx+1, besty, fxp1);
    double subx = subpixel_offset(fxm1, fx0, fxp1);

    // subpixel correction along Y (keep X fixed)
    double fym1=-2, fy0=-2, fyp1=-2;
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty-1, fym1);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty,   fy0);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty+1, fyp1);
    double suby = subpixel_offset(fym1, fy0, fyp1);

    PairMatch out;
    out.dx = double(bestx) + subx;
    out.dy = double(besty) + suby;
    out.score = best; // ZNCC in [-1..1]
    out.ok = std::isfinite(out.dx) && std::isfinite(out.dy) && std::isfinite(out.score);
    return out;
}

} // namespace semstitch
