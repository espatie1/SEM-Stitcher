#include "utils.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

cv::Mat displayize(const cv::Mat& src8u) {
    // src8u ожидается CV_8UC1 (но свалимся в клон если другое)
    if (src8u.empty()) return {};
    if (src8u.type() != CV_8UC1) return src8u.clone();

    double mn, mx;
    cv::minMaxLoc(src8u, &mn, &mx);
    if (mx <= mn) return src8u;

    cv::Mat tmp, dst;
    src8u.convertTo(tmp, CV_8U, 255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
    tmp.copyTo(dst);
    return dst;
}

void save_mosaic_png(const cv::Mat& m, const std::string& path) {
    if (m.empty()) {
        std::cout << "[save] mosaic is empty, nothing to save\n";
        return;
    }
    if (cv::imwrite(path, m)) {
        std::cout << "[save] mosaic saved to " << path
                  << " (" << m.cols << "x" << m.rows << ")\n";
    } else {
        std::cout << "[save] failed to save " << path << "\n";
    }
}
