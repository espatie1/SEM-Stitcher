#include "CpuBackend.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace semstitch {

namespace {
/* Wrap frame data into a cv::Mat and convert to float in [0..1].
   No extra copy for the 8-bit buffer: we reference external data,
   which must stay valid during this call. */
cv::Mat toFloatMat(const Frame& f) {
    // No copy: use external bytes (valid for the duration of this call)
    cv::Mat m8(f.height, f.width, CV_8UC1,
               const_cast<std::uint8_t*>(f.data.data()));
    cv::Mat m32;
    m8.convertTo(m32, CV_32F, 1.0 / 255.0);
    return m32;
}
} // namespace

/* Estimate frame-to-frame drift (translation) with phase correlation.
   Preconditions:
     - prev and curr have the same width/height
     - pixel format is 8-bit grayscale (CV_8UC1)
   Returns: subpixel shift (dx, dy) in pixels. */
DriftVector CpuBackend::drift(const Frame& prev, const Frame& curr) {
    if (prev.width != curr.width || prev.height != curr.height) {
        throw std::runtime_error("CpuBackend::drift: frame sizes differ");
    }

    cv::Mat a = toFloatMat(prev);
    cv::Mat b = toFloatMat(curr);

    // Hanning window improves robustness of phaseCorrelate (less edge ringing)
    cv::Mat win;
    cv::createHanningWindow(win, a.size(), CV_32F);

    cv::Point2d shift = cv::phaseCorrelate(a, b, win);
    return { shift.x, shift.y };
}

/* Compute a simple homography that represents only translation.
   For now, we build it directly from the drift (no rotation/scale/shear). */
Homography CpuBackend::match(const Frame& prev, const Frame& curr) {
    // For now we use pure translational homography derived from drift
    const DriftVector d = drift(prev, curr);

    Homography H{};
    H.h[0] = 1.0; H.h[1] = 0.0; H.h[2] = d.dx;
    H.h[3] = 0.0; H.h[4] = 1.0; H.h[5] = d.dy;
    H.h[6] = 0.0; H.h[7] = 0.0; H.h[8] = 1.0;
    return H;
}

} // namespace semstitch
