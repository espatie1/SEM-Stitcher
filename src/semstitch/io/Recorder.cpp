#include "semstitch/io/Recorder.hpp"

#include <cstring>
#include <stdexcept>

namespace semstitch {

namespace {
#pragma pack(push, 1)
struct FileHeader {
    char     magic[4];   // 'S','S','T','1'
    uint32_t version;    // 1
};
struct RecordHeader {
    uint32_t width;         // 4
    uint32_t height;        // 4
    uint8_t  format;        // 1
    uint8_t  pad[3];        // 3 (делаем кратно 4; с pack(1) не обязательно, но оставим для стабильности)
    uint64_t timestamp_ns;  // 8
    uint64_t seq;           // 8
    uint32_t crc32;         // 4
    uint32_t data_size;     // 4
}; // ИТОГО: 36 байт при pack(1)
#pragma pack(pop)

static_assert(sizeof(FileHeader)   == 8,  "FileHeader size unexpected");
static_assert(sizeof(RecordHeader) == 36, "RecordHeader size unexpected");

} // namespace

//---------------- FrameRecorder ----------------

FrameRecorder::FrameRecorder(const std::string& path)
    : ofs_(path, std::ios::binary)
{
    if (!ofs_) return;

    FileHeader h{};
    h.magic[0] = 'S'; h.magic[1] = 'S'; h.magic[2] = 'T'; h.magic[3] = '1';
    h.version  = 1u;
    ofs_.write(reinterpret_cast<const char*>(&h), sizeof(h));
    ok_ = static_cast<bool>(ofs_);
}

FrameRecorder::~FrameRecorder() = default;

bool FrameRecorder::write(const Frame& f, std::uint64_t seq, std::uint32_t crc32)
{
    if (!ok_) return false;

    RecordHeader rh{};
    rh.width        = f.width;
    rh.height       = f.height;
    rh.format       = static_cast<std::uint8_t>(f.format);
    rh.pad[0] = rh.pad[1] = rh.pad[2] = 0;
    rh.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        f.timestamp.time_since_epoch()).count();
    rh.seq          = seq;
    rh.crc32        = crc32;
    rh.data_size    = static_cast<std::uint32_t>(f.bytes());

    ofs_.write(reinterpret_cast<const char*>(&rh), sizeof(rh));
    if (!ofs_) return false;

    if (rh.data_size) {
        ofs_.write(reinterpret_cast<const char*>(f.data.data()), rh.data_size);
        if (!ofs_) return false;
    }
    return true;
}

//---------------- FramePlayer ----------------

FramePlayer::FramePlayer(const std::string& path)
    : ifs_(path, std::ios::binary)
{
    if (!ifs_) return;
    FileHeader h{};
    ifs_.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (!ifs_) return;
    if (std::memcmp(h.magic, "SST1", 4) != 0 || h.version != 1u) {
        return;
    }
    ok_ = true;
}

FramePlayer::~FramePlayer() = default;

bool FramePlayer::readNext(Frame& out, std::vector<std::uint8_t>& scratch,
                           std::uint64_t* out_seq,
                           std::uint64_t* out_ts_ns,
                           std::uint32_t* out_crc32)
{
    if (!ok_) return false;

    RecordHeader rh{};
    ifs_.read(reinterpret_cast<char*>(&rh), sizeof(rh));
    if (!ifs_) return false;

    scratch.resize(rh.data_size);
    if (rh.data_size) {
        ifs_.read(reinterpret_cast<char*>(scratch.data()), rh.data_size);
        if (!ifs_) return false;
    }

    if (out_seq)    *out_seq    = rh.seq;
    if (out_ts_ns)  *out_ts_ns  = rh.timestamp_ns;
    if (out_crc32)  *out_crc32  = rh.crc32;

    out.width  = rh.width;
    out.height = rh.height;
    out.format = static_cast<PixelFormat>(rh.format);
    out.timestamp = std::chrono::steady_clock::time_point{}; // player сам задаёт ритм
    out.data   = std::span<const std::uint8_t>(scratch.data(), scratch.size());

    return true;
}

} // namespace semstitch
