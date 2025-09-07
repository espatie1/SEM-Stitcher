#pragma once
#include <opencv2/core.hpp>
#include <string>

cv::Mat displayize(const cv::Mat& src8u);
void save_mosaic_png(const cv::Mat& m, const std::string& path);
