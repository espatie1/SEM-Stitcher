#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>

namespace semstitch {

struct ComposeOptions {
    bool   use_hann_weight      = true;   // окном Ханна приглушаем края тайла
    bool   normalize_each_tile  = false;  // подгонять яркость тайла к уже набранной мозаике
    double norm_clip_low        = 0.7;    // ограничение gain
    double norm_clip_high       = 1.3;
    double eps                  = 1e-6;   // числ. стабильность
};

struct ComposeResult {
    cv::Mat  mosaic8u;   // конечная 8-битная мозаика (Gray)
    cv::Size size;       // размер полотна
    cv::Point2d origin;  // какой сдвиг был применён (чтобы все >=0)
};

/** Скомпоновать мозаику по тайлам (Gray8) и их позам (в пикселях, абсолютные). */
ComposeResult composeMosaic(const std::vector<cv::Mat>& tilesGray8,
                            const std::vector<cv::Point2d>& poses,
                            const ComposeOptions& opt = {});

} // namespace semstitch
