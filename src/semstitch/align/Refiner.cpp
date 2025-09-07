#include "semstitch/align/Refiner.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace semstitch {

static bool validROI(const cv::Mat& m, const cv::Rect& r) {
    return r.x>=0 && r.y>=0 && r.x+r.width<=m.cols && r.y+r.height<=m.rows && r.width>0 && r.height>0;
}

/** ZNCC(A,B) на общей части при целочисленном сдвиге B на (dx,dy) относительно A. */
static bool zncc_at_shift(const cv::Mat& A8, const cv::Mat& B8,
                          const cv::Rect& roiA, const cv::Rect& roiB,
                          int dx, int dy, double& out)
{
    CV_Assert(A8.type()==CV_8UC1 && B8.type()==CV_8UC1);
    // локальные одинаковые окна
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
    if (sA < 1e-6 || sB < 1e-6) { out = -1.0; return true; }

    cv::Mat Af, Bf;
    A.convertTo(Af, CV_32F);
    B.convertTo(Bf, CV_32F);
    Af = Af - float(meanA[0]);
    Bf = Bf - float(meanB[0]);

    double num = cv::sum(Af.mul(Bf))[0];
    double den = (double(A.total()) - 1.0) * sA * sB;
    out = num / den;
    return true;
}

/** субпиксельная поправка (классическая параболическая интерполяция 1D) */
static double subpixel_offset(double fm1, double f0, double fp1) {
    double denom = (fm1 - 2.0*f0 + fp1);
    if (std::abs(denom) < 1e-12) return 0.0;
    double delta = 0.5 * (fm1 - fp1) / denom; // в диапазоне примерно [-0.5;0.5] при "нормальных" поверхностях
    if (!std::isfinite(delta)) return 0.0;
    if (delta < -1.0) delta = -1.0;
    if (delta >  1.0) delta =  1.0;
    return delta;
}

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

    // стартовая целочисленная точка
    int cx = int(std::round(init.x));
    int cy = int(std::round(init.y));

    double best = -2.0;
    int bestx = cx, besty = cy;

    // «хилл-клаймб» в окрестности (простой полный перебор в малом радиусе)
    for (int dy = -radius_px; dy <= radius_px; ++dy) {
        for (int dx = -radius_px; dx <= radius_px; ++dx) {
            double s = -2.0;
            if (!zncc_at_shift(a_gray8, b_gray8, roiA, roiB, cx+dx, cy+dy, s)) continue;
            if (s > best) { best = s; bestx = cx+dx; besty = cy+dy; }
        }
    }

    // субпиксельная поправка по X (оставляя Y фиксированным)
    double fxm1=-2, fx0=-2, fxp1=-2;
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx-1, besty, fxm1);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx,   besty, fx0);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx+1, besty, fxp1);
    double subx = subpixel_offset(fxm1, fx0, fxp1);

    // по Y (оставляя X фиксированным)
    double fym1=-2, fy0=-2, fyp1=-2;
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty-1, fym1);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty,   fy0);
    zncc_at_shift(a_gray8, b_gray8, roiA, roiB, bestx, besty+1, fyp1);
    double suby = subpixel_offset(fym1, fy0, fyp1);

    PairMatch out;
    out.dx = double(bestx) + subx;
    out.dy = double(besty) + suby;
    out.score = best; // ZNCC [-1..1]
    out.ok = std::isfinite(out.dx) && std::isfinite(out.dy) && std::isfinite(out.score);
    return out;
}

} // namespace semstitch
