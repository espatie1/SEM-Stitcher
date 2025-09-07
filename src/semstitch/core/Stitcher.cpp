#include "semstitch/core/Stitcher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>

namespace semstitch {

Stitcher::Stitcher(IBackend& backend)
    : Stitcher(backend, Options{}) {}

Stitcher::Stitcher(IBackend& backend, const Options& opt)
    : backend_(backend), opt_(opt) {}

static cv::Mat toGray8View(const Frame& f) {
    CV_Assert(f.format == PixelFormat::Gray8);
    return cv::Mat((int)f.height, (int)f.width, CV_8UC1,
                   const_cast<std::uint8_t*>(f.data.data()));
}

void Stitcher::pushFrame(const Frame& f) {
    if (f.format != PixelFormat::Gray8 || f.width == 0 || f.height == 0) {
        // поддерживаем только 8-битный серый и валидные размеры
        return;
    }

    cv::Mat view = toGray8View(f);
    cv::Mat tile = view.clone(); // отделяемся от внешнего буфера

    std::lock_guard<std::mutex> lk(mtx_);

    cv::Point2d pose(0.0, 0.0);
    if (!haveLast_) {
        // первый тайл — в (0,0)
        pose = cv::Point2d(0.0, 0.0);
        haveLast_ = true;
    } else {
        // инкрементальная оценка сдвига
        cv::Point2d d = estimateOffset(lastTile_, tile);
        pose = lastPose_ + d;
    }

    tiles_.push_back(std::move(tile));
    poses_.push_back(pose);
    lastTile_ = tiles_.back();
    lastPose_ = pose;

    enforceHistoryLimitLocked();
}

cv::Point2d Stitcher::estimateOffset(const cv::Mat& prev8u, const cv::Mat& curr8u) const {
    CV_Assert(prev8u.size() == curr8u.size());
    cv::Mat a32, b32;
    prev8u.convertTo(a32, CV_32F, 1.0/255.0);
    curr8u.convertTo(b32, CV_32F, 1.0/255.0);

    if (opt_.useApodForPhase) {
        cv::Mat winX(1, a32.cols, CV_32F), winY(a32.rows, 1, CV_32F);
        for (int x=0; x<winX.cols; ++x) {
            float phi = 2.f*float(CV_PI)*float(x)/float(std::max(1,winX.cols-1));
            winX.at<float>(0,x) = 0.5f - 0.5f*std::cos(phi);
        }
        for (int y=0; y<winY.rows; ++y) {
            float phi = 2.f*float(CV_PI)*float(y)/float(std::max(1,winY.rows-1));
            winY.at<float>(y,0) = 0.5f - 0.5f*std::cos(phi);
        }
        cv::Mat win = winY * winX;
        a32 = a32.mul(win);
        b32 = b32.mul(win);
    }

    cv::Point2d shift = cv::phaseCorrelate(a32, b32);
    // phaseCorrelate возвращает «как сдвинуть b, чтобы совпало с a»
    // нужно смещение prev->curr → берём отрицательное
    return cv::Point2d(-shift.x, -shift.y);
}

void Stitcher::enforceHistoryLimitLocked() {
    const int lim = std::max(1, opt_.maxTiles);
    if ((int)tiles_.size() <= lim) return;

    const int drop = (int)tiles_.size() - lim;
    tiles_.erase(tiles_.begin(), tiles_.begin() + drop);
    poses_.erase(poses_.begin(), poses_.begin() + drop);

    lastTile_ = tiles_.back();
    lastPose_ = poses_.back();
}

cv::Mat Stitcher::snapshot() const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (tiles_.empty()) return cv::Mat();

    ComposeResult res = composeMosaic(tiles_, poses_, opt_.compose);
    return res.mosaic8u; // CV_8UC1
}

void Stitcher::setComposeOptions(const ComposeOptions& co) {
    std::lock_guard<std::mutex> lk(mtx_);
    opt_.compose = co;
}

} // namespace semstitch
