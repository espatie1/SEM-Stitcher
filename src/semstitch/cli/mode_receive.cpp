#include "modes.hpp"
#include "args.hpp"
#include "utils.hpp"
#include "jitter_buffer.hpp"

#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Stitcher.hpp"
#include "semstitch/io/GrpcClient.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cctype>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <random>
#include <sstream>
#include <cmath>

#ifndef SEMSTITCH_VERSION
#define SEMSTITCH_VERSION "dev"
#endif
#ifndef SEMSTITCH_GIT_SHA
#define SEMSTITCH_GIT_SHA "unknown"
#endif

// ------------ маленькие утилиты для манифеста ------------
static std::string iso_utc_now() {
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
static std::string rand_id() {
    std::mt19937_64 rng{std::random_device{}()};
    std::uniform_int_distribution<unsigned long long> d;
    std::ostringstream os;
    os << std::hex << d(rng) << d(rng);
    return os.str();
}
static std::string jesc(const std::string& s) {
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
static std::string join_argv(int argc, char** argv) {
    std::ostringstream os;
    for (int i=0;i<argc;++i) {
        if (i) os<<' ';
        os<<argv[i];
    }
    return os.str();
}
static void write_text_file(const std::filesystem::path& p, const std::string& txt) {
    std::filesystem::create_directories(p.parent_path());
    std::ofstream f(p, std::ios::binary);
    f << txt;
}
static std::string default_outdir(const std::string& mode) {
    std::string stamp = iso_utc_now(); // 2025-09-07T12:34:56Z
    for (auto& c: stamp) if (c==':' || c=='-') c = '_';
    return std::string("runs/") + mode + "_" + stamp + "_" + rand_id();
}
// минимальный CRC32 для артефактов
static std::uint32_t crc32_update(std::uint32_t crc, const unsigned char* buf, std::size_t len) {
    static std::uint32_t table[256]; static bool init=false;
    if (!init) {
        for (std::uint32_t i=0;i<256;++i){ std::uint32_t c=i;
            for (int k=0;k<8;++k) c = (c&1) ? (0xEDB88320U ^ (c>>1)) : (c>>1);
            table[i]=c;
        }
        init=true;
    }
    crc ^= 0xFFFFFFFFU;
    for (std::size_t i=0;i<len;++i) crc = table[(crc ^ buf[i]) & 0xFFU] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFU;
}
static std::string crc32_file_hex(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    if (!f) return "";
    std::vector<unsigned char> buf(1<<20);
    std::uint32_t crc = 0;
    while (f) {
        f.read(reinterpret_cast<char*>(buf.data()), buf.size());
        std::streamsize got = f.gcount();
        if (got <= 0) break;
        crc = crc32_update(crc, buf.data(), static_cast<std::size_t>(got));
    }
    std::ostringstream os;
    os<<std::hex<<std::uppercase<<std::setfill('0')<<std::setw(8)<<crc;
    return os.str();
}

// =========================================================

int run_receive(int argc, char** argv) {
    using namespace std::chrono;

    const std::string net  = argValue(argc, argv, "net", "balanced");

    // 1) профиль сети → опции клиента
    semstitch::GrpcClient::Options co{};
    if (net == "fast") {
        co.reconnectInitialMs = 200; co.reconnectMaxMs = 2000; co.idleLogMs = 4000; co.enableCompression = false; co.printHeartbeat = true;
    } else if (net == "robust") {
        co.reconnectInitialMs = 300; co.reconnectMaxMs = 8000; co.idleLogMs = 2000; co.enableCompression = true;  co.printHeartbeat = true;
    } else { // balanced
        co.reconnectInitialMs = 250; co.reconnectMaxMs = 5000; co.idleLogMs = 2500; co.enableCompression = true;  co.printHeartbeat = true;
    }

    // 2) приёмные параметры
    const std::string serverAddr = argValue(argc, argv, "server", "localhost:50051");
    std::atomic<int> targetLatencyMs{ std::max(0, argValueInt(argc, argv, "latency", 200)) };
    const int bufcap             = std::max(8, argValueInt(argc, argv, "bufcap", 256));
    const std::string dropFlag   = argValue(argc, argv, "drop", "oldest");
    const std::string saveName   = argValue(argc, argv, "save", "mosaic.png");
    const int healthMs           = std::max(250, argValueInt(argc, argv, "health", 2000));
    const bool withView          = argHas(argc, argv, "view");
    const std::string outdir     = argValue(argc, argv, "outdir", default_outdir("receive"));

    std::filesystem::create_directories(outdir);
    const std::filesystem::path mosaic_path = std::filesystem::path(outdir) / saveName;

    JitterBuffer::DropPolicy dropPolicy =
        (dropFlag == "newest") ? JitterBuffer::DropPolicy::Newest
                               : JitterBuffer::DropPolicy::Oldest;

    const std::string runId   = rand_id();
    const std::string started = iso_utc_now();
    const std::string mode    = "receive";

    std::cout << "[receive] starting…\n";
    std::cout << "[receive] net=" << net << ", server=" << serverAddr
              << ", latency=" << targetLatencyMs.load() << "ms"
              << ", bufcap=" << bufcap << ", drop=" << (dropPolicy==JitterBuffer::DropPolicy::Oldest?"oldest":"newest")
              << (withView ? ", view=on" : ", view=off")
              << ", outdir=" << outdir
              << "\n";

    // 3) stitcher
    auto backend  = semstitch::makeBackend(semstitch::BackendType::CPU);
    semstitch::Stitcher stitch(*backend);

    // 4) джиттер-буфер и метрики
    JitterBuffer jbuf(static_cast<std::size_t>(bufcap), dropPolicy);
    std::atomic<bool> running{true};
    std::atomic<bool> paused{false};
    std::atomic<std::uint64_t> recvFrames{0}, usedFrames{0}, dropCount{0};
    std::atomic<std::uint64_t> recvBytes{0};
    std::atomic<double> ewmaDtMs{0.0};
    std::atomic<std::uint64_t> lastArriveNs{0};

    // буфер последнего кадра для viewer
    std::mutex last_mtx;
    std::vector<std::uint8_t> last_buf;
    std::uint32_t last_w=0, last_h=0;
    semstitch::PixelFormat last_fmt = semstitch::PixelFormat::Gray8;

    const auto t0 = steady_clock::now();

    // 5) запуск gRPC клиента: producer → jbuf
    semstitch::GrpcClient client(serverAddr, co);
    client.start([&](const semstitch::Frame& f) {
        if (f.width == 0 || f.height == 0) return; // heartbeat

        auto now = std::chrono::steady_clock::now();
        std::uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        std::uint64_t prev_ns = lastArriveNs.exchange(now_ns);
        if (prev_ns != 0) {
            double dt = double(now_ns - prev_ns) / 1e6; // ms
            double alpha = 0.1;
            double old = ewmaDtMs.load();
            if (old <= 0.0) ewmaDtMs.store(dt);
            else ewmaDtMs.store(alpha*dt + (1.0-alpha)*old);
        }

        RecvFrame rf;
        rf.width  = f.width;
        rf.height = f.height;
        rf.fmt    = f.format;
        rf.ts_ns  = std::chrono::duration_cast<std::chrono::nanoseconds>(f.timestamp.time_since_epoch()).count();
        rf.t_arrive = now;
        rf.buf.assign(f.data.begin(), f.data.end());

        {   // last-frame для viewer
            std::lock_guard<std::mutex> lk(last_mtx);
            last_buf = rf.buf;
            last_w = rf.width; last_h = rf.height; last_fmt = rf.fmt;
        }

        recvBytes += rf.buf.size();
        ++recvFrames;

        jbuf.push(std::move(rf), dropCount);
    });

    // 6) consumer-тред
    std::thread consumer([&](){
        auto desiredQLen = [&](double avgDtMs)->std::size_t {
            if (avgDtMs <= 0.0) return 1;
            std::size_t q = static_cast<std::size_t>(std::round(std::max(1.0, targetLatencyMs.load() / avgDtMs)));
            return std::min<std::size_t>(q, static_cast<std::size_t>(bufcap));
        };

        while (running) {
            if (paused) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            double avg = ewmaDtMs.load();
            std::size_t want = desiredQLen(avg);
            std::size_t have = jbuf.size();
            if (have < want) continue;

            RecvFrame rf;
            if (jbuf.pop_wait(rf, /*wait_ms*/2)) {
                semstitch::Frame view{
                    std::span<const std::uint8_t>(rf.buf.data(), rf.buf.size()),
                    rf.width, rf.height,
                    std::chrono::steady_clock::now(),
                    rf.fmt
                };
                stitch.pushFrame(view);
                ++usedFrames;

                if (avg > 0.0) {
                    auto nap = std::chrono::milliseconds( std::max(0, int(std::round(avg)) - 1) );
                    std::this_thread::sleep_for(nap);
                }
            }
        }
    });

    // 7) health-лог
    std::thread health([&](){
        using clk = std::chrono::steady_clock;
        auto t0h = clk::now();
        std::uint64_t lastRecv  = 0;
        std::uint64_t lastUsed  = 0;
        std::uint64_t lastBytes = 0;

        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(healthMs));
            auto t1h = clk::now();
            double secs = std::chrono::duration<double>(t1h - t0h).count();
            t0h = t1h;

            std::uint64_t r  = recvFrames.load();
            std::uint64_t u  = usedFrames.load();
            std::uint64_t by = recvBytes.load();

            double fps_in  = (r - lastRecv) / secs;
            double fps_out = (u - lastUsed) / secs;
            double mbytes  = (by - lastBytes) / secs / (1024.0*1024.0);

            lastRecv  = r;
            lastUsed  = u;
            lastBytes = by;

            std::cout << "[health] in_fps=" << std::round(fps_in*10)/10
                      << " out_fps=" << std::round(fps_out*10)/10
                      << " qlen=" << jbuf.size()
                      << " drops=" << dropCount.load()
                      << " bitrate=" << std::round(mbytes*10)/10 << " MB/s"
                      << " avg_dt=" << std::round(ewmaDtMs.load()*10)/10 << " ms"
                      << " latency_target=" << targetLatencyMs.load() << " ms"
                      << (paused ? " [paused]" : "")
                      << "\n";
        }
    });

    // 8) viewer (опционально)
    std::thread viewer;
    if (withView) {
        viewer = std::thread([&](){
            try {
                cv::namedWindow("SEM Viewer", cv::WINDOW_NORMAL);
                cv::resizeWindow("SEM Viewer", 900, 900);
            } catch (...) {
                std::cerr << "[view] failed to create window; running headless.\n";
                return;
            }

            bool showMosaic = true;
            auto overlay = [&](cv::Mat& img){
                int y = 22, dy = 22;
                auto put = [&](const std::string& s){
                    cv::putText(img, s, {10,y}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255), 1, cv::LINE_AA);
                    y += dy;
                };
                put("IN: " + std::to_string(int(std::round((recvFrames.load())))) +
                    "  OUT: " + std::to_string(int(std::round((usedFrames.load())))));
                put("QLEN: " + std::to_string(jbuf.size()) +
                    "  DROPS: " + std::to_string(dropCount.load()));
                put("AVG_DT: " + std::to_string(int(std::round(ewmaDtMs.load()))) + " ms" +
                    "  LATENCY: " + std::to_string(targetLatencyMs.load()) + " ms");
                put("[M] mosaic/frame   [SPACE] pause   [+/-] latency   [S] save   [Q] quit");
            };

            while (running) {
                cv::Mat canvas;

                if (showMosaic) {
                    canvas = stitch.snapshot();
                    if (canvas.empty()) {
                        canvas = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
                        cv::putText(canvas, "Waiting for frames…", {40, 240},
                                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2, cv::LINE_AA);
                    }
                } else {
                    std::vector<std::uint8_t> buf;
                    std::uint32_t w=0,h=0;
                    {
                        std::lock_guard<std::mutex> lk(last_mtx);
                        buf = last_buf; w=last_w; h=last_h;
                    }
                    if (buf.empty() || w==0 || h==0) {
                        canvas = cv::Mat(480, 640, CV_8UC1, cv::Scalar(0));
                        cv::putText(canvas, "No frame yet…", {120, 240},
                                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2, cv::LINE_AA);
                    } else {
                        canvas = cv::Mat(h, w, CV_8UC1, buf.data()).clone();
                    }
                }

                if (canvas.channels() == 1) {
                    cv::Mat shown;
                    double scale = 1.0;
                    const int maxSide = 1000;
                    int side = std::max(canvas.cols, canvas.rows);
                    if (side > maxSide) scale = double(maxSide) / side;
                    cv::resize(canvas, shown, cv::Size(), scale, scale, cv::INTER_AREA);
                    overlay(shown);
                    cv::imshow("SEM Viewer", displayize(shown));
                } else {
                    overlay(canvas);
                    cv::imshow("SEM Viewer", displayize(canvas));
                }

                int key = cv::waitKey(1);
                if (key < 0) continue;
                key = std::tolower(key);
                if (key == 'q' || key == 27) { // ESC
                    running = false;
                    break;
                } else if (key == 's') {
                    save_mosaic_png(stitch.snapshot(), mosaic_path.string());
                } else if (key == 'm') {
                    showMosaic = !showMosaic;
                } else if (key == ' ') {
                    paused = !paused;
                } else if (key == '+' || key == '=') {
                    targetLatencyMs.store(std::min(2000, targetLatencyMs.load() + 50));
                } else if (key == '-' || key == '_') {
                    targetLatencyMs.store(std::max(0, targetLatencyMs.load() - 50));
                }
            }
            cv::destroyWindow("SEM Viewer");
        });
    }

    if (!withView) {
        std::cout << "[receive] press Enter to stop.\n";
        std::cin.get();
        running = false;
    } else {
        viewer.join();
    }

    client.shutdown();
    running = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // итоговое сохранение мозаики в outdir
    save_mosaic_png(stitch.snapshot(), mosaic_path.string());

    // ---------- manifest JSON ----------
    try {
        const std::string finished = iso_utc_now();
        const double dur_s = std::max(1e-6, duration<double>(steady_clock::now() - t0).count());
        const double mbps   = (recvBytes.load() / dur_s) / (1024.0*1024.0);

        // Артефакты
        std::uintmax_t mosaic_sz = 0;
        std::string mosaic_crc;
        if (std::filesystem::exists(mosaic_path)) {
            mosaic_sz  = std::filesystem::file_size(mosaic_path);
            mosaic_crc = crc32_file_hex(mosaic_path);
        }

        std::ostringstream j;
        j << "{\n";
        j << "  \"mist_version\": \"0.1\",\n";
        j << "  \"run_id\": \"" << runId << "\",\n";
        j << "  \"mode\": \"" << mode << "\",\n";
        j << "  \"started_at\": \"" << started << "\",\n";
        j << "  \"finished_at\": \"" << finished << "\",\n";
        j << "  \"outdir\": \"" << jesc(outdir) << "\",\n";
        j << "  \"cli\": { \"argv\": \"" << jesc(join_argv(argc, argv)) << "\" },\n";
        j << "  \"software\": {\n";
        j << "    \"semstitch_version\": \"" << SEMSTITCH_VERSION << "\",\n";
        j << "    \"git_sha\": \"" << SEMSTITCH_GIT_SHA << "\",\n";
        j << "    \"opencv\": \"" << CV_VERSION << "\"\n";
        j << "  },\n";
        j << "  \"input\": { \"grpc_address\": \"" << jesc(serverAddr) << "\" },\n";
        j << "  \"network\": { \"profile\": \"" << net << "\", \"latency_target_ms\": " << targetLatencyMs.load()
          << ", \"bufcap\": " << bufcap << ", \"drop\": \"" << (dropPolicy==JitterBuffer::DropPolicy::Oldest?"oldest":"newest") << "\" },\n";
        j << "  \"stats\": { \"frames_in\": " << recvFrames.load()
          << ", \"frames_out\": " << usedFrames.load()
          << ", \"drops\": " << dropCount.load()
          << ", \"bytes_in\": " << recvBytes.load()
          << ", \"duration_s\": " << std::fixed << std::setprecision(3) << dur_s
          << ", \"mb_per_s\": " << std::fixed << std::setprecision(2) << mbps << " },\n";
        j << "  \"artifacts\": [\n";
        j << "    {\"path\":\"" << jesc(mosaic_path.string()) << "\","
          << "\"size\":" << mosaic_sz << ","
          << "\"sha256\":\"\","
          << "\"crc32\":\"" << mosaic_crc << "\","
          << "\"kind\":\"mosaic\"}\n";
        j << "  ]\n";
        j << "}\n";

        const auto manifest_path = std::filesystem::path(outdir) / "manifest.json";
        write_text_file(manifest_path, j.str());
        std::cout << "[receive] manifest saved: " << manifest_path << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[receive] failed to write manifest: " << e.what() << "\n";
    }

    return 0;
}
