#pragma once
#include "semstitch/core/Frame.hpp"

#include <opencv2/core.hpp>
#include <optional>
#include <random>
#include <chrono>
#include <string>
#include <vector>

namespace semstitch {

/**
 * Источник кадров, который «сканирует» окно (tile) по мастер-изображению
 * (либо загруженному с диска, либо синтетическому паттерну).
 *
 * Поддерживает: зигзагообразный обход сетки, перекрытие, дрейф, джиттер, фликер.
 */
class ViewportScannerSource {
public:
    struct Options {
        int    tileW = 512, tileH = 512;       // размер кадра
        int    gridCols = 5, gridRows = 5;     // сетка прохода
        double overlap = 0.20;                 // перекрытие 0..1
        double driftX = 5.0, driftY = 2.5;     // дрейф (пикс/сек)
        double jitterSigma = 0.4;              // RMS джиттера (пикс)
        double flickerAmp  = 0.03;             // амплитуда фликера яркости (0..1)
        int    masterW = 4096, masterH = 4096; // размер синтетической сцены, если файла нет
        unsigned seed = 0;                     // RNG seed (0 = авто)

        // Новое:
        std::string masterPath{};              // если задан — загрузим этот файл в качестве мастера
        std::string pattern = "grid";          // "grid", "rings", "text:SEM", "checker", "noise"
    };

    // ВАЖНО: без default-аргумента — во избежание ошибки GCC12
    explicit ViewportScannerSource(const Options& opt);
    // Делегирующий конструктор по умолчанию
    ViewportScannerSource();

    std::optional<Frame> next();               // очередной кадр

private:
    Options opt_;
    cv::Mat master_;                           // 8UC1
    std::vector<std::uint8_t> scratch_;        // буфер возврата

    int curIdx_ = 0;                           // текущий индекс плитки
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};
    std::chrono::steady_clock::time_point t0_;

    void ensureMasterReady();
    cv::Mat makePattern(int w, int h, const std::string& pat); // 8UC1
};

} // namespace semstitch
