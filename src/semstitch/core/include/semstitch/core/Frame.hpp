//====================================================================
// File: core/include/semstitch/core/Frame.hpp
//====================================================================
#pragma once


#include <span>
#include <cstdint>
#include <chrono>


namespace semstitch {


/// Pixel storage layouts that the pipeline currently understands.
enum class PixelFormat : std::uint8_t {
Gray8 = 0, ///< 8-bit grayscale, 1 byte per pixel
RGB24, ///< 24-bit RGB, 3 bytes per pixel, interleaved
RGBA32 ///< 32-bit RGBA, 4 bytes per pixel
};


/// Lightweight view of a single SEM frame.
struct Frame {
std::span<const std::uint8_t> data{}; ///< read-only pixel buffer
std::uint32_t width{0};
std::uint32_t height{0};
std::chrono::steady_clock::time_point timestamp{};
PixelFormat format{PixelFormat::Gray8};


/// Total byte size – удобная проверка целостности буфера.
[[nodiscard]] std::size_t bytes() const noexcept {
switch (format) {
case PixelFormat::Gray8: return static_cast<std::size_t>(width) * height;
case PixelFormat::RGB24: return static_cast<std::size_t>(width) * height * 3;
case PixelFormat::RGBA32: return static_cast<std::size_t>(width) * height * 4;
default: return 0;
}
}
};


} // namespace semstitch