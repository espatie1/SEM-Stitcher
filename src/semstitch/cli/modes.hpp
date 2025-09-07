#pragma once

/* Entry points for CLI modes.
   Each function parses argv options and runs the selected mode.
   Return value: 0 on success, non-zero on error. */

/* Simulate a frame source and stream frames via gRPC.
   Example:
     sem-stitch-cli simulate --net=balanced --fps=15 --port=50051 */
int run_simulate(int argc, char** argv);

/* Receive frames via gRPC, build a live mosaic, optional viewer.
   Example:
     sem-stitch-cli receive --server=localhost:50051 --view --save=mosaic.png */
int run_receive (int argc, char** argv);

/* Play frames from a folder (or stream out), and optionally build a mosaic.
   Example:
     sem-stitch-cli play --folder=./imgs --fps=30 --view --save=mosaic.png */
int run_play    (int argc, char** argv);
