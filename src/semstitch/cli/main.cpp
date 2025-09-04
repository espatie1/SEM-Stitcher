#include "semstitch/core/Backend.hpp"
#include "semstitch/core/Stitcher.hpp"
#include "semstitch/io/GrpcServer.hpp"
#include "semstitch/io/GrpcClient.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: sem-stitch-cli <simulate|receive>\n";
        return 0;
    }
    std::string mode(argv[1]);

    if (mode == "receive") {
        auto backend  = semstitch::makeBackend(semstitch::BackendType::CPU);
        semstitch::Stitcher stitch(*backend);

        semstitch::GrpcClient client("localhost:50051");
        client.start([&stitch](const semstitch::Frame& f) {
            stitch.pushFrame(f);
            if (stitch.frameCount() % 30 == 0)
                std::cout << "frames: " << stitch.frameCount() << "\r" << std::flush;
        });

        std::cin.get();  
        client.shutdown();
    } else {
        std::cout << "Unknown mode\n";
    }
}
