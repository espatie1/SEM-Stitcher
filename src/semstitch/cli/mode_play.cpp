// src/semstitch/cli/mode_play.cpp
#include "modes.hpp"
#include "args.hpp"
#include "utils.hpp"

#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Stitcher.hpp"
#include "semstitch/io/GrpcServer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <climits>

// ---------------- helpers ----------------

static bool ieq(const std::string& a, const std::string& b) {
    if (a.size()!=b.size()) return false;
    for (size_t i=0;i<a.size();++i) if (std::tolower(a[i])!=std::tolower(b[i])) return false;
    return true;
}

static std::vector<std::string> split_list(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c: s) {
        if (c==',' || c==';' || std::isspace(static_cast<unsigned char>(c))) {
            if (!cur.empty()) { out.push_back(cur); cur.clear(); }
        } else cur.push_back(c);
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

static std::string ext_of(const std::filesystem::path& p) {
    std::string e = p.extension().string();
    if (!e.empty() && e[0]=='.') e.erase(0,1);
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c){ return std::tolower(c); });
    return e;
}

static std::vector<std::filesystem::path>
list_images_in_folder(const std::filesystem::path& folder,
                      const std::vector<std::string>& allow_exts)
{
    std::vector<std::filesystem::path> files;
    if (!std::filesystem::exists(folder)) return files;
    for (auto& de : std::filesystem::directory_iterator(folder)) {
        if (!de.is_regular_file()) continue;
        auto e = ext_of(de.path());
        if (allow_exts.empty()) {
            files.push_back(de.path());
        } else {
            for (auto& a: allow_exts) {
                if (ieq(e, a)) { files.push_back(de.path()); break; }
            }
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

// Преобразование любого поддерживаемого формата в CV_8UC1
static bool load_as_gray8(const std::filesystem::path& p,
                          std::vector<std::uint8_t>& out,
                          std::uint32_t& w, std::uint32_t& h)
{
    cv::Mat src = cv::imread(p.string(), cv::IMREAD_UNCHANGED);
    if (src.empty()) return false;

    cv::Mat gray;
    if (src.channels() == 1) {
        gray = src;
    } else {
        int code = (src.channels()==4) ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY;
        cv::cvtColor(src, gray, code);
    }

    cv::Mat gray32f;
    switch (gray.depth()) {
        case CV_8U:  gray.convertTo(gray32f, CV_32F, 1.0/255.0);    break;
        case CV_16U: gray.convertTo(gray32f, CV_32F, 1.0/65535.0);  break;
        case CV_32F: gray32f = gray;                                break;
        default:
            gray.convertTo(gray32f, CV_32F);
            break;
    }

    double mn=0.0, mx=0.0;
    cv::minMaxLoc(gray32f, &mn, &mx);
    cv::Mat u8;
    if (mx > mn) {
        gray32f.convertTo(u8, CV_8U, 255.0/(mx-mn), -mn*255.0/(mx-mn));
    } else {
        u8 = cv::Mat(gray32f.size(), CV_8U, cv::Scalar(0));
    }

    w = static_cast<std::uint32_t>(u8.cols);
    h = static_cast<std::uint32_t>(u8.rows);
    out.assign(u8.datastart, u8.dataend);
    return true;
}

// ---------------- main mode ----------------

int run_play(int argc, char** argv)
{
    const std::string file   = argValue(argc, argv, "file", "");     // .sst (пока отключено)
    const std::string folder = argValue(argc, argv, "folder", "");   // папка с кадрами
    const std::string extstr = argValue(argc, argv, "ext", "png,jpg,jpeg,tif,tiff,bmp");
    const bool withView      = argHas(argc, argv, "view");
    const int fps            = std::max(1, argValueInt(argc, argv, "fps", 30));
    const int port           = argValueInt(argc, argv, "port", -1);  // >=0 → стрим наружу gRPC
    const std::string save   = argValue(argc, argv, "save", "mosaic.png");

    // новые флаги длительного проигрывания
    const bool loopFlag      = argHas(argc, argv, "loop");
    const int  repeatArg     = argValueInt(argc, argv, "repeat", 1);
    const int  durationSec   = std::max(0, argValueInt(argc, argv, "duration", 0)); // секунд

    int repeat = std::max(1, repeatArg);
    if (loopFlag) repeat = INT_MAX; // условно бесконечно, пока не остановят вручную или по duration

    if (folder.empty() && file.empty()) {
        std::cerr
            << "[play] usage:\n"
            << "  sem-stitch-cli play --folder=DIR [--ext=png,jpg,tif] [--view] [--save=mosaic.png]\n"
            << "  sem-stitch-cli play --folder=DIR [--fps=30] --port=50051 [--loop|--repeat=N|--duration=S]\n";
        return 1;
    }

    if (!file.empty()) {
        std::cerr << "[play] .sst воспроизведение отключено в этой сборке. Используйте --folder.\n";
        return 1;
    }

    const auto exts  = split_list(extstr);
    const auto files = list_images_in_folder(folder, exts);
    if (files.empty()) {
        std::cerr << "[play] no images found in '" << folder << "' with ext: " << extstr << "\n";
        return 1;
    }

    std::cout << "[play] found " << files.size() << " images in " << folder
              << " (ext=" << extstr << ")\n";

    // ---------- STREAM OUT VIA gRPC ----------
    if (port >= 0) {
        semstitch::GrpcServer::Options so{};
        so.maxQueue = 128; so.dropOldest = true; so.heartbeatMs = 1000; so.enableCompression = true;

        semstitch::GrpcServer server(static_cast<std::uint16_t>(port), so);
        std::cout << "[play] streaming to 0.0.0.0:" << port << " at " << fps << " fps\n";

        const auto frameDelay = std::chrono::milliseconds(1000 / fps);
        std::vector<std::uint8_t> buf;
        std::uint32_t w=0, h=0;

        std::atomic<bool> running{true};
        // запуск фонового ожидания Enter — удобно для бесконечного стрима
        if (loopFlag || repeat>1 || durationSec>0) {
            std::thread([&running](){
                std::cout << "[play] press Enter to stop.\n";
                std::cin.get();
                running = false;
            }).detach();
        }

        auto t0 = std::chrono::steady_clock::now();
        int sent=0;
        int loops=0;

        while (running) {
            for (auto& p : files) {
                if (!running) break;

                if (!load_as_gray8(p, buf, w, h)) {
                    std::cerr << "[play] failed to read '" << p.string() << "' — skipping\n";
                    continue;
                }

                semstitch::Frame f{
                    std::span<const std::uint8_t>(buf.data(), buf.size()),
                    w, h,
                    std::chrono::steady_clock::now(),
                    semstitch::PixelFormat::Gray8
                };
                server.pushFrame(f);

                if ((++sent % fps) == 0) {
                    std::cout << "[play] streamed frames: " << sent
                              << "  (loop " << (loops+1) << ")\n";
                }

                if (durationSec > 0) {
                    auto dt = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::steady_clock::now() - t0).count();
                    if (dt >= durationSec) { running = false; break; }
                }

                std::this_thread::sleep_for(frameDelay);
            }
            ++loops;
            if (!loopFlag && durationSec==0 && loops >= repeat) break;
        }

        std::cout << "[play] done. total streamed: " << sent
                  << "  loops: " << loops << "\n";
        return 0;
    }

    // ---------- LOCAL MOSAIC BUILD ----------
    auto backend  = semstitch::makeBackend(semstitch::BackendType::CPU);
    semstitch::Stitcher stitch(*backend);

    std::vector<std::uint8_t> buf;
    std::uint32_t w=0, h=0;

    if (withView) {
        try {
            cv::namedWindow("SEM Player", cv::WINDOW_NORMAL);
            cv::resizeWindow("SEM Player", 900, 900);
        } catch (...) {
            std::cerr << "[play] failed to create window; continue headless.\n";
        }
    }

    const auto frameDelay = std::chrono::milliseconds(1000 / fps);
    int used=0;

    int loops = 0;
    auto t0 = std::chrono::steady_clock::now();
    bool running = true;

    while (running) {
        for (auto& p : files) {
            if (!load_as_gray8(p, buf, w, h)) {
                std::cerr << "[play] failed to read '" << p.string() << "' — skipping\n";
                continue;
            }

            semstitch::Frame f{
                std::span<const std::uint8_t>(buf.data(), buf.size()),
                w, h,
                std::chrono::steady_clock::now(),
                semstitch::PixelFormat::Gray8
            };
            stitch.pushFrame(f);
            ++used;

            if (withView) {
                cv::Mat snap = stitch.snapshot();
                if (snap.empty()) {
                    cv::Mat empty(480, 640, CV_8UC1, cv::Scalar(0));
                    cv::putText(empty, "Waiting…", {40,240}, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255), 2, cv::LINE_AA);
                    cv::imshow("SEM Player", displayize(empty));
                } else {
                    cv::Mat shown;
                    double scale = 1.0;
                    int side = std::max(snap.cols, snap.rows);
                    if (side > 1000) scale = 1000.0 / side;
                    cv::resize(snap, shown, cv::Size(), scale, scale, cv::INTER_AREA);
                    cv::imshow("SEM Player", displayize(shown));
                }
                int k = cv::waitKey(1);
                if (k==27 || k=='q' || k=='Q') { running=false; break; }
            }

            if (durationSec > 0) {
                auto dt = std::chrono::duration_cast<std::chrono::seconds>(
                              std::chrono::steady_clock::now() - t0).count();
                if (dt >= durationSec) { running = false; break; }
            }

            std::this_thread::sleep_for(frameDelay);
            if (used % 30 == 0) std::cout << "[play] consumed: " << used << "\n";
        }
        ++loops;
        if (!loopFlag && durationSec==0 && loops >= repeat) break;
        if (!running) break;
    }

    save_mosaic_png(stitch.snapshot(), save);
    if (withView) cv::destroyWindow("SEM Player");

    std::cout << "[play] mosaic saved to " << save << "\n";
    return 0;
}
