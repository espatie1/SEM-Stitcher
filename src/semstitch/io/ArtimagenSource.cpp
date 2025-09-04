#include "semstitch/io/ArtimagenSource.hpp"
#include "libartimagen/artimagen.h"

namespace semstitch {

ArtimagenSource::ArtimagenSource(const std::string& cfgPath) {
    engine_ = std::make_unique<artimagen::Engine>(cfgPath);
}

std::optional<Frame> ArtimagenSource::next() {
    auto img = engine_->render();
    if (!img.valid()) return std::nullopt;

    buffer_.assign(img.data(), img.data() + img.size());
    return Frame{
        std::span<const std::uint8_t>(buffer_.data(), buffer_.size()),
        img.width(),
        img.height(),
        std::chrono::steady_clock::now(),
        PixelFormat::Gray8
    };
}

} // namespace semstitch
