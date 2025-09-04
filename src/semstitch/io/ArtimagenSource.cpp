#include "semstitch/io/ArtimagenSource.hpp"
#include "libartimagen/artimagen.h"    // C-API типов/структур/прототипов
#include "libartimagen/artimagen_i.h"  // artimagen::CImage и AIGApp

#include <chrono>
#include <stdexcept>
#include <cmath>
#include <iostream>   // лог

namespace {

// Логгер сообщений ARTIMAGEN (прогресс/ошибки)
void AigLogCallback(t_message* m) {
    if (!m) return;
    std::cerr << "[artimagen] "
              << (m->sender_id ? m->sender_id : "?")
              << " msg=" << m->message;
    if (m->comment) std::cerr << " : " << m->comment;
    std::cerr << std::endl;
}

} // namespace

namespace semstitch {

ArtimagenSource::ArtimagenSource(const std::string& /*luaSceneFile*/)
{
    // 0) Инициализируем глобальный объект приложения ARTIMAGEN (если не создан библиотекой)
    if (!AIGApp) {
        AIGApp = new artimagen::CApp();
        AIGApp->set_message_call_back(&AigLogCallback);
        std::cerr << "[artimagen] CApp created\n";
    }

    // 1) Базовые параметры «пробы» (gold on carbon)
    gcDef_ = {};
    gcDef_.sizex = 5000;
    gcDef_.sizey = 5000;
    gcDef_.ee_coefficient = 1.0f;
    gcDef_.ee_top_above_base = 0.25;
    gcDef_.base_level = 0.35;
    gcDef_.base_level_variation = 0.05;
    gcDef_.grain_min_size = 20;
    gcDef_.grain_max_size = 80;
    gcDef_.number_of_grains = 350;
    gcDef_.rotation = 0.0f;
    gcDef_.fs_density = 0.002f;
    gcDef_.fs_min_r = 2;
    gcDef_.fs_max_r = 5;
    gcDef_.fs_min_coe = 0.9f;
    gcDef_.fs_max_coe = 1.1f;

    // 2) Параметры «сканирования» кадра
    imgDef_ = {};
    imgDef_.sizex = 512;
    imgDef_.sizey = 512;
    imgDef_.bg_min_gl = 0.02;
    imgDef_.bg_max_gl = 0.08;
    imgDef_.bg_dens_x = 16;
    imgDef_.bg_dens_y = 16;
    imgDef_.beam_sigma = 1.2f;
    imgDef_.beam_astig_ratio = 1.0f;
    imgDef_.beam_astig_angle = 0.0f;
    imgDef_.shift_x = 0;
    imgDef_.shift_y = 0;
    imgDef_.vib_pixel_dwell_time = 1000;
    imgDef_.vib_min_frequency = 30.0f;
    imgDef_.vib_max_frequency = 90.0f;
    imgDef_.vib_max_amplitude = 0.1f;
    imgDef_.vib_number_of_frequencies = 2;
    imgDef_.vib_pixel_dead_time = 0;
    imgDef_.vib_line_dead_time = 0;
    imgDef_.noise_sigma = 0.005;
    imgDef_.contrast = 1.0;
    imgDef_.brightness = 0.0;

    // 3) Создаём виртуальную «пробу»
    sample_ = generate_gc_sample(&gcDef_);
    if (!sample_) {
        std::cerr << "[artimagen] generate_gc_sample() returned null\n";
        throw std::runtime_error("ARTIMAGEN: generate_gc_sample() failed");
    }
    std::cerr << "[artimagen] sample created\n";
}

ArtimagenSource::~ArtimagenSource()
{
    if (sample_) {
        destroy_gc_sample(sample_);
        sample_ = nullptr;
    }
}

std::optional<Frame> ArtimagenSource::next()
{
    // Рендер кадра → CImage*
    void* raw = generate_standard_image(sample_, &imgDef_);
    if (!raw) {
        std::cerr << "[artimagen] generate_standard_image() returned null\n";
        return std::nullopt;
    }

    auto* img = static_cast<artimagen::CImage*>(raw);
    auto* buf = img->get_buffer(); // IM_STORE_TYPE* == double*
    const auto w  = static_cast<uint32_t>(img->get_sizex());
    const auto h  = static_cast<uint32_t>(img->get_sizey());
    const std::size_t sz = static_cast<std::size_t>(w) * h;

    // Конвертируем [0..1] double → [0..255] uint8_t
    scratch_.resize(sz);
    for (std::size_t i = 0; i < sz; ++i) {
        double v = buf[i];
        if (v < 0.0) v = 0.0;
        if (v > 1.0) v = 1.0;
        scratch_[i] = static_cast<std::uint8_t>(std::lround(v * 255.0));
    }

    // Освобождаем изображение, выданное ARTIMAGEN
    aig_destroy_image(raw);

    return Frame{
        std::span<const std::uint8_t>(scratch_.data(), scratch_.size()),
        w, h,
        std::chrono::steady_clock::now(),
        PixelFormat::Gray8
    };
}

} // namespace semstitch
