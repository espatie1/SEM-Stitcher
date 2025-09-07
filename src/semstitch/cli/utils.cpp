#include "utils.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

/*
  Prepare an image for on-screen display.

  - Expects CV_8UC1 (8-bit grayscale). If the type is different,
    we simply clone and return it unchanged.
  - If the image has no contrast (max <= min), return as-is.
  - Otherwise, linearly rescale pixel values to [0..255].
*/
cv::Mat displayize(const cv::Mat& src8u) {
    // Expected type is CV_8UC1; for other types, just return a clone
    if (src8u.empty()) return {};
    if (src8u.type() != CV_8UC1) return src8u.clone();

    double mn, mx;
    cv::minMaxLoc(src8u, &mn, &mx);
    if (mx <= mn) return src8u;

    // Linear contrast stretch to the full 8-bit range
    cv::Mat tmp, dst;
    src8u.convertTo(tmp, CV_8U, 255.0 / (mx - mn), -mn * 255.0 / (mx - mn));
    tmp.copyTo(dst);
    return dst;
}

/*
  Save a mosaic as a PNG image.

  - If the mosaic is empty, do nothing (print a message).
  - On success/failure, print a log line with the path and size.
*/
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
