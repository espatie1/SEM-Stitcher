#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Stitcher.hpp"
#include "semstitch/io/ArtimagenSource.hpp"
#include "semstitch/io/GrpcServer.hpp"
#include "semstitch/io/GrpcClient.hpp"

#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <vector>

// ---- утилита для парсинга простых флагов вида --key=value
static std::string argValue(int argc, char** argv, const std::string& key, const std::string& def = {}) {
    const std::string pref = "--" + key + "=";
    for (int i = 2; i < argc; ++i) { // начиная со 2-го аргумента (после режима)
        std::string a(argv[i]);
        if (a.rfind(pref, 0) == 0) return a.substr(pref.size());
    }
    return def;
}
static int argValueInt(int argc, char** argv, const std::string& key, int def) {
    try {
        std::string v = argValue(argc, argv, key, "");
        if (v.empty()) return def;
        return std::stoi(v);
    } catch (...) { return def; }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout
            << "Usage:\n"
            << "  sem-stitch-cli simulate [--net=fast|balanced|robust] [--fps=30] [--port=50051]\n"
            << "  sem-stitch-cli receive  [--net=fast|balanced|robust] [--server=localhost:50051]\n";
        return 0;
    }

    const std::string mode = argv[1];
    const std::string net  = argValue(argc, argv, "net", "balanced");

    // ---------- SIMULATE ----------
    if (mode == "simulate") {
        // профиль сети → опции сервера
        semstitch::GrpcServer::Options so{};
        if (net == "fast") {
            so.max_queue = 32;  so.dropOldest = true;  so.heartbeatMs = 1500; so.enableCompression = false;
        } else if (net == "robust") {
            so.max_queue = 512; so.dropOldest = true;  so.heartbeatMs = 750;  so.enableCompression = true;
        } else { // balanced
            so.max_queue = 128; so.dropOldest = true;  so.heartbeatMs = 1000; so.enableCompression = true;
        }

        const int port = argValueInt(argc, argv, "port", 50051);
        const int fps  = std::max(1, argValueInt(argc, argv, "fps", 30));
        const auto frameDelay = std::chrono::milliseconds(1000 / fps);

        std::cout << "[simulate] starting…\n";
        std::cout << "[simulate] net=" << net << ", fps=" << fps << ", port=" << port << "\n";

        try {
            semstitch::ArtimagenSource src("resources/config/artimagen/default.yaml");
            semstitch::GrpcServer      server(static_cast<std::uint16_t>(port), so);

            int sent = 0;
            while (auto f = src.next()) {
                server.pushFrame(*f);
                if ((++sent % fps) == 0) {
                    std::cout << "[simulate] sent frames: " << sent << "\n";
                }
                std::this_thread::sleep_for(frameDelay);
            }
            std::cout << "[simulate] finished. total sent: " << sent << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[simulate] error: " << e.what() << '\n';
            return 1;
        }
        return 0;
    }

    // ---------- RECEIVE ----------
    if (mode == "receive") {
        // профиль сети → опции клиента
        semstitch::GrpcClient::Options co{};
        if (net == "fast") {
            co.reconnectInitialMs = 200; co.reconnectMaxMs = 2000; co.idleLogMs = 4000; co.enableCompression = false;
        } else if (net == "robust") {
            co.reconnectInitialMs = 300; co.reconnectMaxMs = 8000; co.idleLogMs = 2000; co.enableCompression = true;
        } else { // balanced
            co.reconnectInitialMs = 250; co.reconnectMaxMs = 5000; co.idleLogMs = 2500; co.enableCompression = true;
        }

        const std::string serverAddr = argValue(argc, argv, "server", "localhost:50051");

        std::cout << "[receive] starting…\n";
        std::cout << "[receive] net=" << net << ", server=" << serverAddr << "\n";

        auto backend  = semstitch::makeBackend(semstitch::BackendType::CPU);
        semstitch::Stitcher stitch(*backend);

        semstitch::GrpcClient client(serverAddr, co);
        std::atomic<int> frames{0};

        client.start([&](const semstitch::Frame& f) {
            // heartbeat: width/height == 0 → просто игнорируем
            if (f.width == 0 || f.height == 0) return;

            stitch.pushFrame(f);
            int n = ++frames;
            if (n % 30 == 0) {
                std::cout << "[grpc-client] received frames: " << n << "\n";
            }
        });

        std::cout << "[receive] press Enter to stop.\n";
        std::cin.get();

        client.shutdown();

        // сохраним итоговую мозаику
        auto mosaic = stitch.snapshot();
        if (!mosaic.empty()) {
            if (cv::imwrite("mosaic.png", mosaic)) {
                std::cout << "[receive] mosaic saved to mosaic.png (" << mosaic.cols << "x" << mosaic.rows << ")\n";
            } else {
                std::cout << "[receive] failed to save mosaic.png\n";
            }
        } else {
            std::cout << "[receive] no mosaic to save (no frames?)\n";
        }
        return 0;
    }

    std::cout << "Unknown mode: " << mode << "\n";
    return 0;
}
