#include "semstitch/io/ArtimagenSource.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace semstitch {

// --- Снижаем болтливость сообщений библиотеки
void ArtimagenSource::msgCallback(t_message* m) {
    if (!m) return;
    switch (m->message) {
        case AIG_MSG_FATAL_ERROR:
        case AIG_MSG_OOPS:
            std::cerr << "[artimagen] " << (m->sender_id ? m->sender_id : "?")
                      << " FATAL: " << (m->comment ? m->comment : "") << "\n";
            break;
        case AIG_MSG_CREATING:
        case AIG_MSG_APPLYING:
        case AIG_MSG_SAVING: {
            static int skip = 0;
            if ((skip++ % 50) == 0) {
                std::cout << "[artimagen] " << (m->sender_id ? m->sender_id : "?")
                          << " msg=" << m->message
                          << " : " << (m->comment ? m->comment : "") << "\n";
            }
        } break;
        default:
            // подавляем шум
            break;
    }
}

ArtimagenSource::ArtimagenSource(const std::string& /*luaSceneFile*/) {
    // Конфиг логов/потоков библиотеки
    if (AIGApp) {
        AIGApp->set_message_call_back(&ArtimagenSource::msgCallback);
        AIGApp->set_number_of_threads(std::max(2u, std::thread::hardware_concurrency()));
    }

    // ---- Параметры выборки (упрощённые ради скорости старта)
    gcDef_.sizex = 3000;                 // [nm]
    gcDef_.sizey = 3000;
    gcDef_.number_of_grains = 80;        // было 300 — долго
    gcDef_.grain_min_size   = 15;
    gcDef_.grain_max_size   = 60;
    gcDef_.ee_coefficient   = 0.0f;
    gcDef_.fs_density       = 0.0f;

    // ---- Параметры изображения
    imgDef_.sizex = 384;                 // 512 → 384 (быстрее)
    imgDef_.sizey = 384;

    // Фон
    imgDef_.bg_min_gl = 0.10;
    imgDef_.bg_max_gl = 0.25;
    imgDef_.bg_dens_x = 8;
    imgDef_.bg_dens_y = 8;

    // Пучок (PSF)
    imgDef_.beam_sigma        = 0.7f;
    imgDef_.beam_astig_ratio  = 1.0f;
    imgDef_.beam_astig_angle  = 0.0f;

    // Вибрации
    imgDef_.vib_pixel_dwell_time      = 500;
    imgDef_.vib_min_frequency         = 5.0f;
    imgDef_.vib_max_frequency         = 40.0f;
    imgDef_.vib_max_amplitude         = 0.4f;
    imgDef_.vib_number_of_frequencies = 3;
    imgDef_.vib_pixel_dead_time       = 0;
    imgDef_.vib_line_dead_time        = 0;

    // Шум/тон
    imgDef_.noise_sigma = 0.02;
    imgDef_.contrast    = 1.0;
    imgDef_.brightness  = 0.0;

    // ---- Создаём выборку и «прогреваем» планы
    sample_ = generate_gc_sample(&gcDef_);
    if (!sample_) throw std::runtime_error("ARTIMAGEN: generate_gc_sample() failed");

    image_ = generate_standard_image(sample_, &imgDef_);
    if (!image_) throw std::runtime_error("ARTIMAGEN: generate_standard_image() failed (warm-up)");

    // Прогрев завершён — освобождаем, в next() каждый раз создаём новое
    aig_destroy_image(image_);
    image_ = nullptr;

    std::cout << "[artimagen] sample created\n";
}

ArtimagenSource::~ArtimagenSource() {
    if (image_)  { aig_destroy_image(image_); image_  = nullptr; }
    if (sample_) { destroy_gc_sample(sample_); sample_ = nullptr; }
}

std::optional<Frame> ArtimagenSource::next() {
    // На каждый кадр — новое CImage (актуальные данные)
    if (image_) { aig_destroy_image(image_); image_ = nullptr; }

    image_ = generate_standard_image(sample_, &imgDef_);
    if (!image_) return std::nullopt;

    const auto w = static_cast<std::uint32_t>(imgDef_.sizex);
    const auto h = static_cast<std::uint32_t>(imgDef_.sizey);
    const auto N = static_cast<std::size_t>(w) * h;

    // ВАЖНО: get_buffer() неконстантный → используем неконстантный указатель.
    auto* img = static_cast<artimagen::CImage*>(image_);
    const double* buf = img->get_buffer(); // NxM double, диапазон [0..1]

    scratch_.resize(N);
    for (std::size_t i = 0; i < N; ++i) {
        double v = buf[i];
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        scratch_[i] = static_cast<std::uint8_t>(std::lround(v * 255.0));
    }

    // Освобождаем CImage сразу — наружу отдаём только копию в scratch_
    aig_destroy_image(image_);
    image_ = nullptr;

    return Frame{
        std::span<const std::uint8_t>(scratch_.data(), scratch_.size()),
        w, h,
        std::chrono::steady_clock::now(),
        PixelFormat::Gray8
    };
}

} // namespace semstitch
