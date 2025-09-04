#include "semstitch/core/Stitcher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace semstitch {

static Frame wrapMat(const cv::Mat& m) {
    return { std::span<const std::uint8_t>(m.ptr<std::uint8_t>(), m.total()),
             static_cast<std::uint32_t>(m.cols),
             static_cast<std::uint32_t>(m.rows),
             std::chrono::steady_clock::now(),
             PixelFormat::Gray8 };
}

Stitcher::Stitcher(IBackend& backend, const Config& cfg)
    : backend_{backend}, drift_{backend}, cfg_{cfg} {}

void Stitcher::pushFrame(const Frame& frame) {
    std::lock_guard<std::mutex> lock(mtx_);

    cv::Mat curr(frame.height, frame.width, CV_8UC1,
                 const_cast<unsigned char*>(frame.data.data()));

    if (first_) {
        int canvasW = static_cast<int>(frame.width)  * 8;
        int canvasH = static_cast<int>(frame.height) * 8;
        mosaic_ = cv::Mat::zeros(canvasH, canvasW, CV_8UC1);

        int x0 = (canvasW - frame.width) / 2;
        int y0 = (canvasH - frame.height) / 2;
        curr.copyTo(mosaic_(cv::Rect(x0, y0, frame.width, frame.height)));

        offsetX_ = static_cast<double>(x0);
        offsetY_ = static_cast<double>(y0);
        lastFrame_ = curr.clone();
        first_ = false;
        ++framesProcessed_;
        return;
    }
    Frame prevF = wrapMat(lastFrame_);
    DriftVector dv = drift_(prevF, frame);

    offsetX_ += dv.dx;
    offsetY_ += dv.dy;

    int x = static_cast<int>(std::round(offsetX_));
    int y = static_cast<int>(std::round(offsetY_));

    if (x >= 0 && y >= 0 &&
        x + frame.width  <= mosaic_.cols &&
        y + frame.height <= mosaic_.rows) {
        curr.copyTo(mosaic_(cv::Rect(x, y, frame.width, frame.height)));
    }

    lastFrame_ = curr.clone();
    ++framesProcessed_;
}

cv::Mat Stitcher::snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return mosaic_.clone();
}

} // namespace semstitch

