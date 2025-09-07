#pragma once

#include "semstitch/core/Frame.hpp"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace semstitch {

// Очень простой бинарный формат SST1:
//
// Header:
//   char     magic[4] = 'S','S','T','1'
//   uint32_t version  = 1
//
// Для каждого кадра — Record:
//   uint32_t width
//   uint32_t height
//   uint8_t  format (PixelFormat)
//   uint8_t  _pad[3] = {0,0,0}
//   uint64_t timestamp_ns
//   uint64_t seq
//   uint32_t crc32
//   uint32_t data_size
//   uint8_t  data[data_size]
//
// Всё little-endian (как на x86/amd64).

class FrameRecorder {
public:
    explicit FrameRecorder(const std::string& path);
    ~FrameRecorder();

    // crc32 и seq — то, что мы уже считаем на стороне сервера/клиента;
    // если не нужны, можно передавать 0.
    bool write(const Frame& f, std::uint64_t seq, std::uint32_t crc32);

private:
    std::ofstream ofs_;
    bool ok_{false};
};

class FramePlayer {
public:
    explicit FramePlayer(const std::string& path);
    ~FramePlayer();

    // Читает следующий кадр.
    // Буфер данных возвращается через scratch, чтобы Frame мог ссылаться на него.
    // Возвращает false — конец файла или ошибка.
    bool readNext(Frame& out, std::vector<std::uint8_t>& scratch,
                  std::uint64_t* out_seq = nullptr,
                  std::uint64_t* out_ts_ns = nullptr,
                  std::uint32_t* out_crc32 = nullptr);

private:
    std::ifstream ifs_;
    bool ok_{false};
};

} // namespace semstitch
