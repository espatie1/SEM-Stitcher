#include "modes.hpp"
#include "args.hpp"

#include <iostream>
#include <string>

static void print_usage() {
    std::cout
        << "Usage:\n"
        << "  sem-stitch-cli simulate [--net=fast|balanced|robust] [--fps=30] [--port=50051] [--record=out.sst]\n"
        << "  sem-stitch-cli receive  [--net=fast|balanced|robust] [--server=localhost:50051]\n"
        << "                          [--latency=200] [--bufcap=256] [--drop=oldest|newest]\n"
        << "                          [--health=2000] [--save=mosaic.png] [--view]\n"
        << "  sem-stitch-cli play     --file=in.sst [--fps=30|--realtime] [--port=50051] [--save=mosaic.png]\n"
        << "     play без --port делает локальную сборку мозаики и сохраняет PNG;\n"
        << "     play с --port играет файл наружу по gRPC (как simulate).\n";
}

int main(int argc, char** argv)
{
    if (argc < 2) { print_usage(); return 0; }
    const std::string mode = argv[1];

    if      (mode == "simulate") return run_simulate(argc, argv);
    else if (mode == "receive")  return run_receive (argc, argv);
    else if (mode == "play")     return run_play    (argc, argv);

    std::cout << "Unknown mode: " << mode << "\n";
    print_usage();
    return 0;
}
