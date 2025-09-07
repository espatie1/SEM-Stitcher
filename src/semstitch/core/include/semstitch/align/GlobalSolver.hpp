#pragma once
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include "semstitch/align/GridAlign.hpp"

namespace semstitch {

/** Параметры решателя. */
struct SolveOptions {
    int    max_iters        = 15;     // максимум итераций IRLS
    double huber_delta_px   = 3.0;    // порог Huber (px) для векторного резидуала (dx,dy)
    double anchor_weight    = 1e6;    // «жесткость» фиксации (0,0) для тайла (0,0)
    double zncc_pow         = 2.0;    // усиливать вес хороших ZNCC: w ~ (max(0,zncc))^zncc_pow
    double min_use_zncc     = 0.10;   // игнорировать ребра с ZNCC ниже этого
    double phase_resp_min   = 0.01;   // очень слабый нижний порог для response фаз-корр
    double stop_eps         = 1e-4;   // относительная остановка по ||delta||
};

/** Результат глобального решения. */
struct SolveResult {
    std::vector<cv::Point2d> poses;   // (x,y) для тайлов в row-major: r*cols+c
    int    iters{0};
    double rmse_px{0.0};              // RMSE по всем использованным связям
    int    used_edges{0};
    int    inliers{0};
    int    outliers{0};
    std::string report;               // короткий текст-лог
};

/**
 * Глобальное выравнивание по трансляциям.
 * rows, cols — размер сетки; edges берём из computePairMatches().
 * Возвращает позиции (x,y) всех тайлов, где (0,0) заякорен в (0,0).
 */
SolveResult solveGlobalTranslation(const AlignResult& matches,
                                   int rows, int cols,
                                   const SolveOptions& opt = {});

/** Удобная инициализация поз по медианным шагам (dx по «right», dy по «down»). */
std::vector<cv::Point2d> initializeByMedians(const AlignResult& matches,
                                             int rows, int cols);

} // namespace semstitch
