#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Stitcher.hpp"
#include "semstitch/io/ArtimagenSource.hpp"
#include "semstitch/io/GrpcServer.hpp"
#include "semstitch/io/GrpcClient.hpp"

#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: sem-stitch-cli <simulate|receive>\n";
        return 0;
    }
    const std::string mode(argv[1]);

    if (mode == "simulate") {
        try {
            std::cerr << "[simulate] starting…\n";
            semstitch::ArtimagenSource src("resources/config/artimagen/default.yaml");
            semstitch::GrpcServer      server(50051);

            std::size_t produced = 0;
            while (auto f = src.next()) {
                server.pushFrame(*f);
                ++produced;
                if (produced % 30 == 0) {
                    std::cerr << "[simulate] produced frames: " << produced << '\n';
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 fps
            }
            std::cerr << "[simulate] source finished.\n";
        } catch (const std::exception& e) {
            std::cerr << "simulate error: " << e.what() << '\n';
            return 1;
        }

    } else if (mode == "receive") {
        std::cerr << "[receive] starting…\n";
        auto backend  = semstitch::makeBackend(semstitch::BackendType::CPU);
        semstitch::Stitcher stitch(*backend);

        semstitch::GrpcClient client("localhost:50051");
        client.start([&stitch](const semstitch::Frame& f) {
            stitch.pushFrame(f);
            if (stitch.frameCount() % 30 == 0)
                std::cout << "frames: " << stitch.frameCount() << "\r" << std::flush;
        });

        std::cerr << "[receive] press Enter to stop.\n";
        std::cin.get();
        client.shutdown();
    } else {
        std::cout << "Unknown mode\n";
    }
}
