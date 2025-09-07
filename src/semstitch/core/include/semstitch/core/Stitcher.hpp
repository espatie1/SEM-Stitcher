#pragma once
#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Frame.hpp"
#include "semstitch/compose/Mosaic.hpp"

#include <opencv2/core.hpp>
#include <mutex>
#include <vector>

namespace semstitch {

/// Инкрементальный ститчер:
/// - принимает кадры (Gray8);
/// - оценивает сдвиг curr относительно prev (phaseCorrelate);
/// - snapshot() вызывает composeMosaic() (Hann-веса + нормализация).
class Stitcher {
public:
    struct Options {
        // ограничение размера истории (чтобы не раздувать память)
        int  maxTiles         = 2000;

        // параметры оценки сдвига
        bool useApodForPhase  = true; // окно Ханна перед phaseCorrelate

        // опции компоновщика
        ComposeOptions compose{
            /*use_hann_weight=*/true,
            /*normalize_each_tile=*/true,
            /*norm_clip_low=*/0.8,
            /*norm_clip_high=*/1.25,
            /*eps=*/1e-6f
        };
    };

    // ВАЖНО: без дефолтного аргумента здесь — иначе GCC ругается
    explicit Stitcher(IBackend& backend);
    Stitcher(IBackend& backend, const Options& opt);

    /// добавить кадр в поток
    void pushFrame(const Frame& f);

    /// получить текущую мозаику (8-бит, один канал)
    cv::Mat snapshot() const;

    /// поменять опции компоновки на лету
    void setComposeOptions(const ComposeOptions& co);

    Options options() const { return opt_; }

private:
    IBackend& backend_;
    Options   opt_;

    // накопленные тайлы и их позы (в координатах мозаики; double — субпиксель)
    std::vector<cv::Mat>     tiles_;   // CV_8UC1
    std::vector<cv::Point2d> poses_;   // левый-верхний угол

    // для инкрементальной оценки
    cv::Mat      lastTile_;            // CV_8UC1
    cv::Point2d  lastPose_{0.0, 0.0};
    bool         haveLast_{false};

    mutable std::mutex mtx_;

    // оценка относительного сдвига curr относительно prev (prev -> curr)
    cv::Point2d estimateOffset(const cv::Mat& prev8u, const cv::Mat& curr8u) const;

    // обрезаем историю, если вышла за лимит
    void enforceHistoryLimitLocked();
};

} // namespace semstitch
