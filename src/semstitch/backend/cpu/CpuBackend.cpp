#include "CpuBackend.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace semstitch {

namespace {
// Оборачиваем данные кадра в Mat и конвертируем в float [0..1]
cv::Mat toFloatMat(const Frame& f) {
    // Без копии: используем внешние данные (валидны в рамках вызова)
    cv::Mat m8(f.height, f.width, CV_8UC1,
               const_cast<std::uint8_t*>(f.data.data()));
    cv::Mat m32;
    m8.convertTo(m32, CV_32F, 1.0 / 255.0);
    return m32;
}
} // namespace

DriftVector CpuBackend::drift(const Frame& prev, const Frame& curr) {
    if (prev.width != curr.width || prev.height != curr.height) {
        throw std::runtime_error("CpuBackend::drift: frame sizes differ");
    }

    cv::Mat a = toFloatMat(prev);
    cv::Mat b = toFloatMat(curr);

    // Окно Хэннинга для устойчивости phaseCorrelation
    cv::Mat win;
    cv::createHanningWindow(win, a.size(), CV_32F);

    cv::Point2d shift = cv::phaseCorrelate(a, b, win);
    return { shift.x, shift.y };
}

Homography CpuBackend::match(const Frame& prev, const Frame& curr) {
    // Пока строим чисто трансляционную гомографию из дрейфа
    const DriftVector d = drift(prev, curr);

    Homography H{};
    H.h[0] = 1.0; H.h[1] = 0.0; H.h[2] = d.dx;
    H.h[3] = 0.0; H.h[4] = 1.0; H.h[5] = d.dy;
    H.h[6] = 0.0; H.h[7] = 0.0; H.h[8] = 1.0;
    return H;
}

} // namespace semstitch
