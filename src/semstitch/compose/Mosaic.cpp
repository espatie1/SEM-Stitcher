#include "semstitch/compose/Mosaic.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace semstitch {

/* Create a 2D Hann window as an outer product of 1D windows.
   Result size: h x w, type: CV_32F, values in [0..1]. */
static cv::Mat makeHann2D(int h, int w) {
    cv::Mat wx(1, w, CV_32F), wy(h, 1, CV_32F);
    for (int x=0; x<w; ++x) {
        float phi = 2.f * float(CV_PI) * float(x) / float(std::max(1, w-1));
        wx.at<float>(0,x) = 0.5f - 0.5f * std::cos(phi);
    }
    for (int y=0; y<h; ++y) {
        float phi = 2.f * float(CV_PI) * float(y) / float(std::max(1, h-1));
        wy.at<float>(y,0) = 0.5f - 0.5f * std::cos(phi);
    }
    return wy * wx; // outer product
}

/* Axis-aligned rectangle for a tile placed at (p.x, p.y). */
static inline cv::Rect2d tileRectAt(const cv::Size& sz, const cv::Point2d& p) {
    return cv::Rect2d(p.x, p.y, sz.width, sz.height);
}

/* Ensure the matrix is a non-empty 8-bit single-channel image. */
static inline void ensureGray8(const cv::Mat& m) {
    CV_Assert(!m.empty());
    CV_Assert(m.type() == CV_8UC1);
}

/*
  Compose a mosaic from grayscale tiles with simple weighted blending.

  Inputs:
    - tilesGray8 : all tiles (CV_8UC1), same size.
    - poses      : per-tile positions (x,y) in pixels (may be subpixel).
    - opt        : options:
        * use_hann_weight       - apply Hann weights per tile
        * normalize_each_tile   - brightness gain match on overlap
        * norm_clip_low/high    - gain clipping range
        * eps                   - small value to avoid division by zero

  Method:
    1) Compute bounding box of all placed tiles (with subpixel shifts).
       Shift the mosaic so that coordinates become non-negative.
    2) Prepare accumulators:
         acc  – weighted sum of pixel values,
         wsum – sum of weights,
         hann2 – per-tile weight image (Hann or ones).
    3) For each tile:
         - convert to float [0..1],
         - multiply by hann2 (weights),
         - build affine for pure translation (tx, ty),
         - (optional) estimate gain on overlap with current mosaic preview,
         - warp weighted tile and weights into accumulators,
           add into acc and wsum.
    4) Finalize:
         mosaic = acc / (wsum + eps) → convert to 8-bit.

  Output:
    - out.mosaic8u : composed mosaic (CV_8UC1),
    - out.size     : mosaic size,
    - out.origin   : original coordinate origin (inverse of the shift).
*/
ComposeResult composeMosaic(const std::vector<cv::Mat>& tilesGray8,
                            const std::vector<cv::Point2d>& poses,
                            const ComposeOptions& opt)
{
    ComposeResult out{};
    const int N = (int)tilesGray8.size();
    CV_Assert(N > 0 && (int)poses.size() == N);
    for (const auto& t : tilesGray8) ensureGray8(t);

    const cv::Size tileSz = tilesGray8[0].size();

    // --- 1) Bounding box of all tiles (including subpixel shift)
    double minX= std::numeric_limits<double>::infinity();
    double minY= std::numeric_limits<double>::infinity();
    double maxX=-std::numeric_limits<double>::infinity();
    double maxY=-std::numeric_limits<double>::infinity();
    for (int i=0;i<N;++i) {
        cv::Rect2d R = tileRectAt(tileSz, poses[i]);
        minX = std::min(minX, R.x);
        minY = std::min(minY, R.y);
        maxX = std::max(maxX, R.x + R.width);
        maxY = std::max(maxY, R.y + R.height);
    }
    // Shift so that everything becomes non-negative [0..)
    const cv::Point2d shift(-std::floor(minX), -std::floor(minY));
    const int outW = (int)std::ceil(maxX + shift.x);
    const int outH = (int)std::ceil(maxY + shift.y);
    CV_Assert(outW>0 && outH>0);

    // --- 2) Accumulators
    cv::Mat acc   = cv::Mat::zeros(outH, outW, CV_32F);
    cv::Mat wsum  = cv::Mat::zeros(outH, outW, CV_32F);
    cv::Mat hann2 = opt.use_hann_weight ? makeHann2D(tileSz.height, tileSz.width)
                                        : cv::Mat(tileSz, CV_32F, cv::Scalar(1));

    // --- 3) Main loop: place tiles into accumulators
    cv::Mat tileF, wimg, tmp, tmpW, mosaicNow, invW;

    for (int i=0;i<N;++i) {
        // Prepare tile: float [0..1]
        tilesGray8[i].convertTo(tileF, CV_32F, 1.0/255.0); // [0..1]
        cv::multiply(tileF, hann2, wimg);                  // apply Hann weights

        // Affine transform for pure translation
        const double tx = poses[i].x + shift.x;
        const double ty = poses[i].y + shift.y;
        cv::Mat M = (cv::Mat_<double>(2,3) << 1,0,tx, 0,1,ty);

        // Brightness normalization relative to the current mosaic (on overlap)
        double gain = 1.0;
        if (opt.normalize_each_tile) {
            // Quick current preview of the mosaic
            invW = wsum + opt.eps;
            cv::divide(acc, invW, mosaicNow);

            // Warp the UNweighted tile and a unit mask to mosaic space
            cv::warpAffine(tileF, tmp,  M, acc.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

            cv::Mat ones(tileSz, CV_32F, cv::Scalar(1));
            cv::warpAffine(ones, tmpW, M, acc.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

            // Overlap masks: (warped ones) > 0  &&  wsum > 0
            cv::Mat m1_8u, m2_8u, overlap8u;
            cv::compare(tmpW, 1e-6, m1_8u, cv::CMP_GT);      // CV_8U
            cv::compare(wsum, 1e-6, m2_8u, cv::CMP_GT);      // CV_8U
            cv::bitwise_and(m1_8u, m2_8u, overlap8u);        // CV_8U

            if (cv::countNonZero(overlap8u) > 0) {
                cv::Scalar mTile  = cv::mean(tmp,       overlap8u);
                cv::Scalar mMosa  = cv::mean(mosaicNow, overlap8u);
                double mt = mTile[0], mm = mMosa[0];
                if (mt > 1e-6) {
                    gain = mm / mt;
                    gain = std::clamp(gain, opt.norm_clip_low, opt.norm_clip_high);
                }
            }
        }

        // Apply gain to the weighted tile and warp into accumulators
        if (gain != 1.0) wimg *= (float)gain;

        cv::warpAffine(wimg,  tmp,  M, acc.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        cv::warpAffine(hann2, tmpW, M, acc.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

        acc  += tmp;
        wsum += tmpW;
    }

    // --- 4) Finalize: divide acc by weights, convert to 8-bit
    cv::Mat mosaicF;
    cv::Mat denom = wsum + opt.eps;
    cv::divide(acc, denom, mosaicF);
    mosaicF.convertTo(out.mosaic8u, CV_8U, 255.0, 0.0);
    out.size   = out.mosaic8u.size();
    out.origin = cv::Point2d(-shift.x, -shift.y); // inverse shift w.r.t. original coordinates
    return out;
}

} // namespace semstitch
