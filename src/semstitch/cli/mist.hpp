#pragma once
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <random>
#include <cstdint>

#if __has_include(<openssl/sha.h>)
  #define MIST_HAVE_OPENSSL 1
  #include <openssl/sha.h>
#else
  #undef MIST_HAVE_OPENSSL
#endif

namespace mist {

struct Artifact {
    std::string path;
    std::uintmax_t size{0};
    std::string sha256;     // может быть пустым, если OpenSSL недоступен
    std::string crc32;      // заполняем как fallback
    std::string kind;       // e.g. "image/png", "record/sst", "mosaic", "debug"
};

inline std::string iso_utc_now() {
    using namespace std::chrono;
    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    std::ostringstream os;
    os << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
    return os.str();
}

inline std::string rand_id() {
    std::mt19937_64 rng{std::random_device{}()};
    std::uniform_int_distribution<unsigned long long> d;
    std::ostringstream os;
    os << std::hex << d(rng) << d(rng);
    return os.str();
}

inline std::string jesc(const std::string& s) {
    std::string o; o.reserve(s.size()+8);
    for (char c: s) {
        switch(c){
            case '\"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default: o += (unsigned char)c < 0x20 ? '?' : c;
        }
    }
    return o;
}

inline void ensure_dir(const std::filesystem::path& p) {
    std::filesystem::create_directories(p);
}
inline void write_text_file(const std::filesystem::path& p, const std::string& txt) {
    ensure_dir(p.parent_path());
    std::ofstream f(p, std::ios::binary);
    f << txt;
}

inline std::uint32_t crc32_update(std::uint32_t crc, const unsigned char* buf, std::size_t len) {
    static std::uint32_t table[256]; static bool init=false;
    if (!init) {
        for (std::uint32_t i=0;i<256;++i){ std::uint32_t c=i;
            for (int k=0;k<8;++k) c = c&1 ? 0xEDB88320U ^ (c>>1) : (c>>1);
            table[i]=c;
        }
        init=true;
    }
    crc ^= 0xFFFFFFFFU;
    for (std::size_t i=0;i<len;++i) crc = table[(crc ^ buf[i]) & 0xFFU] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFU;
}

inline std::string sha256_file(const std::filesystem::path& p, std::string* crc32_hex=nullptr) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return "";
    std::vector<unsigned char> buf(1<<20);
#ifdef MIST_HAVE_OPENSSL
    SHA256_CTX ctx; SHA256_Init(&ctx);
#endif
    std::uint32_t crc = 0;
    while (f) {
        f.read(reinterpret_cast<char*>(buf.data()), buf.size());
        std::streamsize got = f.gcount();
        if (got <= 0) break;
#ifdef MIST_HAVE_OPENSSL
        SHA256_Update(&ctx, buf.data(), static_cast<size_t>(got));
#endif
        crc = crc32_update(crc, buf.data(), static_cast<size_t>(got));
    }
    if (crc32_hex) {
        std::ostringstream os; os<<std::hex<<std::setfill('0')<<std::setw(8)<<std::uppercase<<crc;
        *crc32_hex = os.str();
    }
#ifdef MIST_HAVE_OPENSSL
    unsigned char out[SHA256_DIGEST_LENGTH];
    SHA256_Final(out, &ctx);
    std::ostringstream os;
    os<<std::hex<<std::setfill('0');
    for (unsigned char b: out) os<<std::setw(2)<<static_cast<int>(b);
    return os.str();
#else
    return "";
#endif
}

inline std::string default_outdir(const std::string& mode) {
    auto stamp = iso_utc_now();
    for (auto& c: stamp) if (c==':'||c=='-') c = '_';
    return std::string("runs/") + mode + "_" + stamp + "_" + rand_id();
}

inline std::string join_argv(int argc, char** argv) {
    std::ostringstream os;
    for (int i=0;i<argc;++i) {
        if (i) os<<' ';
        os<<argv[i];
    }
    return os.str();
}

inline std::string os_name() {
#if defined(_WIN32)
    return "Windows";
#elif defined(__APPLE__)
    return "macOS";
#elif defined(__linux__)
    return "Linux";
#else
    return "Unknown";
#endif
}

} // namespace mist
