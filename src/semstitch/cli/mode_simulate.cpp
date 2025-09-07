#include "modes.hpp"
#include "args.hpp"
#include "utils.hpp"

#include "semstitch/io/GrpcServer.hpp"
#include "semstitch/io/ArtimagenSource.hpp"
#include "semstitch/io/ViewportScannerSource.hpp"

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <algorithm>

int run_simulate(int argc, char** argv) {
    const std::string net  = argValue(argc, argv, "net", "balanced");
    const int port         = argValueInt(argc, argv, "port", 50051);
    const int fps          = std::max(1, argValueInt(argc, argv, "fps", 15));
    const std::string sim  = argValue(argc, argv, "sim", "basic"); // basic|scan

    semstitch::GrpcServer::Options so{};
    if (net == "fast")      { so.maxQueue=32;  so.dropOldest=true; so.heartbeatMs=1500; so.enableCompression=false; }
    else if (net=="robust") { so.maxQueue=512; so.dropOldest=true; so.heartbeatMs=750;  so.enableCompression=true;  }
    else                    { so.maxQueue=128; so.dropOldest=true; so.heartbeatMs=1000; so.enableCompression=true;  }

    std::cout << "[simulate] starting…\n";
    std::cout << "[simulate] net=" << net << ", fps=" << fps << ", port=" << port
              << ", mode=" << sim << "\n";

    semstitch::GrpcServer server(static_cast<std::uint16_t>(port), so);
    const auto frameDelay = std::chrono::milliseconds(1000 / fps);

    try {
        std::unique_ptr<semstitch::ArtimagenSource> basic;
        std::unique_ptr<semstitch::ViewportScannerSource> scan;

        if (sim == "scan") {
            semstitch::ViewportScannerSource::Options o{};
            o.tileW = argValueInt(argc, argv, "tilew", 512);
            o.tileH = argValueInt(argc, argv, "tileh", 512);
            {
                // grid=CxR
                std::string g = argValue(argc, argv, "grid", "5x5");
                int xPos = (int)g.find('x');
                if (xPos>0) {
                    o.gridCols = std::max(1, std::stoi(g.substr(0, xPos)));
                    o.gridRows = std::max(1, std::stoi(g.substr(xPos+1)));
                }
            }
            o.overlap      = std::clamp<double>(std::stod(argValue(argc, argv, "overlap", "0.2")), 0.0, 0.95);
            o.driftX       = std::stod(argValue(argc, argv, "driftx", "0.2"));
            o.driftY       = std::stod(argValue(argc, argv, "drifty", "0.1"));
            o.jitterSigma  = std::stod(argValue(argc, argv, "jitter",  "0.3"));
            o.flickerAmp   = std::stod(argValue(argc, argv, "flicker", "0.03"));
            o.masterW      = argValueInt(argc, argv, "masterw", 4096);
            o.masterH      = argValueInt(argc, argv, "masterh", 4096);
            scan = std::make_unique<semstitch::ViewportScannerSource>(o);
        } else {
            basic = std::make_unique<semstitch::ArtimagenSource>("resources/config/artimagen/default.yaml");
        }

        int sent = 0;
        while (true) {
            auto f = (scan ? scan->next() : basic->next());
            if (!f) break;
            server.pushFrame(*f);
            if ((++sent % fps) == 0) std::cout << "[simulate] sent frames: " << sent << "\n";
            std::this_thread::sleep_for(frameDelay);
        }
        std::cout << "[simulate] finished. total sent: " << sent << "\n";
    } catch (const std::exception& e) {
        std::cerr << "[simulate] error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
