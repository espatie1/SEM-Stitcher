#include "semstitch/io/ViewportScannerSource.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace semstitch {

namespace {
constexpr double kPI = 3.1415926535897932384626433832795;
inline std::uint8_t clampU8(int v) { return static_cast<std::uint8_t>(std::max(0, std::min(255, v))); }
} // namespace

ViewportScannerSource::ViewportScannerSource()
    : ViewportScannerSource(Options{}) {}

ViewportScannerSource::ViewportScannerSource(const Options& opt)
    : opt_(opt)
{
    unsigned seed = opt_.seed ? opt_.seed : std::random_device{}();
    rng_.seed(seed);
    t0_ = std::chrono::steady_clock::now();
    ensureMasterReady();
}

void ViewportScannerSource::ensureMasterReady() {
    // 1) Если указан путь к файлу — пробуем загрузить
    if (!opt_.masterPath.empty()) {
        cv::Mat m = cv::imread(opt_.masterPath, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            throw std::runtime_error("ViewportScannerSource: failed to load master image: " + opt_.masterPath);
        }
        if (m.type() != CV_8UC1) {
            cv::Mat g; m.convertTo(g, CV_8U); m = g;
        }
        master_ = m;
        return;
    }

    // 2) Иначе — синтетический паттерн
    master_ = makePattern(opt_.masterW, opt_.masterH, opt_.pattern);
}

cv::Mat ViewportScannerSource::makePattern(int w, int h, const std::string& pat) {
    cv::Mat base(h, w, CV_8UC1);
    // Мягкая «SEM-подложка»
    cv::randn(base, 128, 12);                      // гаусс-шум
    cv::GaussianBlur(base, base, {0,0}, 1.5);      // лёгкий блюр

    auto draw_grid = [&](int step, int thickness, int val){
        for (int y = 0; y < h; y += step) cv::line(base, {0,y}, {w-1,y}, cv::Scalar(val), thickness, cv::LINE_AA);
        for (int x = 0; x < w; x += step) cv::line(base, {x,0}, {x,h-1}, cv::Scalar(val), thickness, cv::LINE_AA);
    };
    auto draw_rings = [&](){
        cv::Point c(w/2, h/2);
        for (int r = 80; r < std::min(w,h)/2; r += 80) {
            cv::circle(base, c, r, cv::Scalar(220), 2, cv::LINE_AA);
        }
    };
    auto draw_text = [&](std::string s){
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 4.0;
        int thick = 5;
        cv::Size sz = cv::getTextSize(s, font, scale, thick, nullptr);
        cv::putText(base, s, {(w - sz.width)/2, (h + sz.height)/2}, font, scale, cv::Scalar(240), thick, cv::LINE_AA);
    };
    auto draw_checker = [&](){
        int cs = 96;
        for (int y = 0; y < h; y += cs) {
            for (int x = 0; x < w; x += cs) {
                bool on = ((x/cs) + (y/cs)) & 1;
                if (on) cv::rectangle(base, {x,y}, {std::min(x+cs,w)-1, std::min(y+cs,h)-1}, cv::Scalar(210), cv::FILLED);
            }
        }
    };

    if (pat.rfind("text:", 0) == 0) {
        draw_grid(128, 1, 180);
        draw_text(pat.substr(5));
        draw_rings();
    } else if (pat == "rings") {
        draw_grid(128, 1, 170);
        draw_rings();
    } else if (pat == "checker") {
        draw_checker();
        draw_rings();
    } else if (pat == "noise") {
        // оставим только подложку
    } else { // "grid" (default)
        draw_grid(128, 1, 180);
        draw_text("SEM");
    }

    // лёгкая нормализация
    double mn, mx; cv::minMaxLoc(base, &mn, &mx);
    if (mx > mn) {
        base.convertTo(base, CV_8U, 255.0/(mx-mn), -mn*255.0/(mx-mn));
    }
    return base;
}

std::optional<Frame> ViewportScannerSource::next() {
    if (master_.empty()) return std::nullopt;

    // Параметры сетки и перекрытия
    const int W = master_.cols, H = master_.rows;
    const int tw = opt_.tileW, th = opt_.tileH;
    const double stepX = tw * (1.0 - opt_.overlap);
    const double stepY = th * (1.0 - opt_.overlap);

    const int tilesX = std::max(1, opt_.gridCols);
    const int tilesY = std::max(1, opt_.gridRows);
    const int total  = tilesX * tilesY;

    int k = curIdx_ % total;
    int gy = k / tilesX;
    int gx = k % tilesX;
    // «змейка»
    if (gy & 1) gx = (tilesX - 1) - gx;

    // Базовая позиция
    double x0 = gx * stepX;
    double y0 = gy * stepY;

    // Дрейф по времени
    using clk = std::chrono::steady_clock;
    double tsec = std::chrono::duration<double>(clk::now() - t0_).count();
    x0 += opt_.driftX * tsec;
    y0 += opt_.driftY * tsec;

    // Джиттер
    x0 += opt_.jitterSigma * gauss_(rng_);
    y0 += opt_.jitterSigma * gauss_(rng_);

    // Ограничим ROI
    int x = std::max(0, std::min((int)std::lround(x0), W - tw));
    int y = std::max(0, std::min((int)std::lround(y0), H - th));

    cv::Rect roi{x, y, tw, th};
    cv::Mat patch = master_(roi).clone();

    // Фликер (медленное изменение яркости)
    double flick = 1.0 + opt_.flickerAmp * std::sin(2.0 * kPI * 0.35 * tsec);
    if (std::abs(flick - 1.0) > 1e-3) {
        cv::Mat tmp; patch.convertTo(tmp, CV_16S); // чтобы не клипать раньше времени
        for (int i = 0; i < tmp.rows; ++i) {
            short* p = tmp.ptr<short>(i);
            for (int j = 0; j < tmp.cols; ++j) {
                int v = (int)std::lround(p[j] * flick);
                p[j] = (short)std::max(-32768, std::min(32767, v));
            }
        }
        tmp.convertTo(patch, CV_8U);
    }

    // Подготовим буфер возврата
    scratch_.assign(patch.data, patch.data + (patch.rows * patch.cols));

    // Следующая плитка
    ++curIdx_;
    if (curIdx_ >= total) curIdx_ = 0;

    return Frame{
        std::span<const std::uint8_t>(scratch_.data(), scratch_.size()),
        static_cast<std::uint32_t>(patch.cols),
        static_cast<std::uint32_t>(patch.rows),
        clk::now(),
        PixelFormat::Gray8
    };
}

} // namespace semstitch
