#include "modes.hpp"
#include "args.hpp"

#include <iostream>
#include <string>

/*
  CLI entry point.

  Modes:
    - simulate : generate frames and stream via gRPC.
    - receive  : connect to a gRPC server, build a live mosaic.
    - play     : read frames from a file/folder or restream them.
*/
static void print_usage() {
    std::cout
        << "Usage:\n"
        << "  sem-stitch-cli simulate [--net=fast|balanced|robust] [--fps=30] [--port=50051] [--record=out.sst]\n"
        << "  sem-stitch-cli receive  [--net=fast|balanced|robust] [--server=localhost:50051]\n"
        << "                          [--latency=200] [--bufcap=256] [--drop=oldest|newest]\n"
        << "                          [--health=2000] [--save=mosaic.png] [--view]\n"
        << "  sem-stitch-cli play     --file=in.sst [--fps=30|--realtime] [--port=50051] [--save=mosaic.png]\n"
        << "     play without --port builds a local mosaic and saves PNG;\n"
        << "     play with --port streams the file via gRPC (like simulate).\n";
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
