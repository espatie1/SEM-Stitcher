# SEM-Stitcher — Technical Documentation

## 1. Overview

SEM-Stitcher is a C++17 toolkit for building live mosaics from SEM (or generic grayscale) frames. It provides:

- Incremental frame ingestion and drift estimation.
- Grid-based pair matching and a robust global translation solver.
- Weighted mosaic composition with optional per-tile brightness normalization.
- gRPC streaming server/client for real-time pipelines.
- Record/playback of a simple binary format (SST1).
- Synthetic sources for testing.

The codebase depends mainly on OpenCV, gRPC/protobuf, and zlib. OpenSSL is optional for SHA-256 in some utilities.

---

## 2. Core Data Model

### 2.1 `PixelFormat`
```cpp
enum class PixelFormat : std::uint8_t {
  Gray8 = 0,  // 1 byte/pixel
  RGB24,      // 3 bytes/pixel, interleaved
  RGBA32      // 4 bytes/pixel
};
```

### 2.2 `Frame`
```cpp
struct Frame {
  std::span<const std::uint8_t> data;       // read-only pixel buffer
  std::uint32_t width{0}, height{0};
  std::chrono::steady_clock::time_point timestamp{};
  PixelFormat format{PixelFormat::Gray8};

  // Byte count based on format; use as an integrity check.
  std::size_t bytes() const noexcept;
};
```
Notes:
- Most pipeline components currently use `Gray8` frames.
- `bytes()` returns 0 for unknown formats and should be checked by callers.

---

## 3. Backends and Motion Estimation

### 3.1 Backend Interface
```cpp
struct DriftVector { double dx{0.0}, dy{0.0}; };
struct Homography  { double h[9] {1,0,0, 0,1,0, 0,0,1}; };

class IBackend {
public:
  virtual ~IBackend() = default;
  virtual DriftVector drift(const Frame& prev, const Frame& curr) = 0;
  virtual Homography  match(const Frame& prev, const Frame& curr) = 0;
};
```

### 3.2 Backend Factory
```cpp
std::unique_ptr<IBackend> makeBackend(BackendType type, std::size_t workerThreads = 0);
```
- `CPU` is implemented.
- `CUDA` and `OpenCL` are not implemented and will throw.

### 3.3 `CpuBackend`
- `drift()` uses `cv::phaseCorrelate` with a full-frame Hanning window.
- `match()` currently returns a pure translation homography based on `drift()`.

---

## 4. Incremental Stitcher

### 4.1 Class
```cpp
class Stitcher {
public:
  struct Options {
    int  maxTiles = 2000;         // history limit
    bool useApodForPhase = true;  // Hanning window before phase correlation
    ComposeOptions compose{true, true, 0.8, 1.25, 1e-6};
  };

  explicit Stitcher(IBackend& backend);
  Stitcher(IBackend& backend, const Options& opt);

  void   pushFrame(const Frame& f);     // append and estimate pose
  cv::Mat snapshot() const;             // compose current mosaic (Gray8)
  void   setComposeOptions(const ComposeOptions& co);
  Options options() const;

private:
  cv::Point2d estimateOffset(const cv::Mat& prev8u, const cv::Mat& curr8u) const;
  void enforceHistoryLimitLocked();
};
```

### 4.2 Behavior
- Accepts `Gray8` frames only; others are ignored.
- The first tile is placed at `(0,0)`.
- For subsequent frames, the relative shift (prev→curr) is estimated by phase correlation (optionally with apodization) and accumulated to produce absolute tile poses.
- `snapshot()` calls the compositor (see §6).

---

## 5. Pair Matching and Alignment

### 5.1 Phase Correlation (`PhaseCorrelator.hpp/.cpp`)
```cpp
struct PairMatch {
  double dx{0.0}, dy{0.0}; // shift of B relative to A: A(x,y) ~ B(x - dx, y - dy)
  double score{0.0};       // response from phaseCorrelate (or ZNCC in refine)
  bool   ok{false};
};

PairMatch phaseCorrelateGray8(const cv::Mat& a_gray8,
                              const cv::Mat& b_gray8,
                              const cv::Rect& roiA,
                              const cv::Rect& roiB,
                              int hanning_win = 16);
```
- Expects `CV_8UC1` inputs and equal ROI sizes.
- If `hanning_win > 0`, uses a 2D Hanning window for edge apodization.

### 5.2 Local Refinement (`Refiner.hpp/.cpp`)
```cpp
PairMatch refineLocalZNCC(const cv::Mat& a_gray8,
                          const cv::Mat& b_gray8,
                          const cv::Rect& roiA,
                          const cv::Rect& roiB,
                          cv::Point2d init,
                          int radius_px = 4,
                          int min_valid_side = 8);
```
- Hill-climb search in a ±`radius_px` window around `init`.
- Computes ZNCC on overlap for integer shifts; then applies 1D parabolic subpixel refinement for X and Y.
- Returns refined `(dx, dy)` and `score = ZNCC`.

### 5.3 Grid Matching (`GridAlign.hpp/.cpp`)
```cpp
struct GridSpec {
  int rows{0}, cols{0};
  double overlap_percent{10.0};       // per-axis overlap in percent
  double stage_repeatability_px{3.0}; // mechanical bounds
};

struct PairEdge {
  int r1{0}, c1{0}, r2{0}, c2{0};
  PairMatch coarse;  // phase correlation result
  PairMatch fine;    // ZNCC refine result
  bool      ok{false};
  std::string direction; // "right" or "down"
};

struct AlignResult { std::vector<PairEdge> edges; };

AlignResult computePairMatches(const std::vector<cv::Mat>& tiles_gray8,
                               const GridSpec& spec,
                               int apod_win = 16,
                               int refine_radius = 4,
                               double phase_resp_thresh = 0.02,
                               double zncc_thresh = 0.20,
                               double bound_factor = 4.0);
```
- For each tile, matches only its right and down neighbors using ROI overlaps determined from `overlap_percent`.
- Applies a bound check: `|dx|, |dy| <= bound_factor * max(1, stage_repeatability_px)`.
- Keeps edges where coarse response ≥ `phase_resp_thresh` and refined ZNCC ≥ `zncc_thresh`.

### 5.4 Global Translation Solver (`GlobalSolver.hpp/.cpp`)
```cpp
struct SolveOptions {
  int    max_iters        = 15;
  double huber_delta_px   = 3.0;   // for residual vector (dx, dy)
  double anchor_weight    = 1e6;   // anchor tile (0,0) at (0,0)
  double zncc_pow         = 2.0;   // conf = (max(0, zncc))^zncc_pow
  double min_use_zncc     = 0.10;  // ignore edges below this ZNCC
  double phase_resp_min   = 0.01;  // coarse response threshold
  double stop_eps         = 1e-4;  // IRLS delta magnitude stop
};

struct SolveResult {
  std::vector<cv::Point2d> poses; // row-major index r*cols + c
  int    iters{0};
  double rmse_px{0.0};
  int    used_edges{0}, inliers{0}, outliers{0};
  std::string report;
};

SolveResult solveGlobalTranslation(const AlignResult& matches,
                                   int rows, int cols,
                                   const SolveOptions& opt = {});

std::vector<cv::Point2d> initializeByMedians(const AlignResult& matches,
                                             int rows, int cols);
```

**Algorithm summary**
- Build linear equations for edges:  
  `x_j - x_i ≈ dx`, `y_j - y_i ≈ dy` for each selected edge.
- Confidence weight per edge:  
  `conf = pow(max(0, zncc), zncc_pow)`; ignore if `< min_use_zncc`.  
  Filter by coarse phase correlation response `>= phase_resp_min`.
- IRLS with Huber weights on residual norm `||r||2`.  
  If `s <= δ`, weight = 1; else `δ/s`.
- Anchor `(x0, y0)` at `(0,0)` with a large diagonal weight (`anchor_weight`).
- Solve normal equations for `δ` using Cholesky → Eigen → SVD fallback.
- Update poses and reweight until `max |δ| < stop_eps` or `max_iters` reached.
- Initialization: `initializeByMedians` uses median `dx` from `"right"` edges and median `dy` from `"down"` edges to set a grid layout.

---

## 6. Mosaic Composition

### 6.1 API
```cpp
struct ComposeOptions {
  bool   use_hann_weight      = true;  // taper tile edges
  bool   normalize_each_tile  = false; // per-tile brightness gain on overlap
  double norm_clip_low        = 0.7;   // gain clamp
  double norm_clip_high       = 1.3;
  double eps                  = 1e-6;  // avoid division by zero
};

struct ComposeResult {
  cv::Mat  mosaic8u;   // CV_8UC1 mosaic
  cv::Size size;       // mosaic size
  cv::Point2d origin;  // applied shift making all coords >= 0
};

ComposeResult composeMosaic(const std::vector<cv::Mat>& tilesGray8,
                            const std::vector<cv::Point2d>& poses,
                            const ComposeOptions& opt = {});
```

### 6.2 Method
- Compute a bounding box of all tiles at `poses`. Shift all coordinates so the mosaic domain starts at `(0,0)`.
- Accumulate:
  - `acc` = sum of weighted tiles.
  - `wsum` = sum of weights.
- Per-tile weight:
  - If `use_hann_weight`, use 2D Hann; else ones.
- Optional brightness normalization:
  - Build a quick preview `mosaicNow = acc / (wsum + eps)`.
  - Warp the unweighted tile and a unit mask to mosaic space. Compute mean tile intensity and mean mosaic intensity on the overlap mask. Compute `gain = mean_mosaic / mean_tile`, then clamp to `[norm_clip_low, norm_clip_high]`.
  - Apply `gain` to the weighted tile before accumulation.
- Final mosaic: `mosaic = acc / (wsum + eps)`, then convert to `CV_8U`.

---

## 7. I/O: Streaming and Files

### 7.1 gRPC Server (`GrpcServer`)
```cpp
class GrpcServer {
public:
  struct Options {
    int   maxQueue = 256;          // bounded queue size
    bool  dropOldest = true;       // drop policy when full
    int   heartbeatMs = 1000;      // idle heartbeat period
    bool  enableCompression = false; // reserved
  };

  explicit GrpcServer(std::uint16_t port = 50051);
  GrpcServer(std::uint16_t port, Options opt);
  ~GrpcServer();

  void pushFrame(const Frame& f);
};
```
Behavior:
- Keeps a deque of outgoing frames. When full, applies drop policy.
- Exposes a bidi RPC `StreamFrames`; the server writes `FrameChunk` messages.
- Sends heartbeat chunks (width=height=0, empty data) when no frames are available.
- Each data chunk includes `seq` (incrementing) and `crc32` of the payload.

### 7.2 gRPC Client (`GrpcClient`)
```cpp
class GrpcClient {
public:
  struct Options {
    int reconnectInitialMs = 300;
    int reconnectMaxMs     = 8000;
    int idleLogMs          = 2500;
    bool enableCompression = false; // reserved
    bool printHeartbeat    = true;
  };

  explicit GrpcClient(const std::string& serverAddr);
  GrpcClient(const std::string& serverAddr, Options opt);
  ~GrpcClient();

  using FrameHandler = std::function<void(const Frame&)>;
  void start(FrameHandler cb);
  void shutdown();
};
```
Behavior:
- Connects to `serverAddr` and reads the `StreamFrames` bidi stream.
- Validates CRC32 and checks sequence continuity. Drops bad/duplicate frames and logs gaps.
- Converts protobuf pixel format to `PixelFormat` and hands frames to `FrameHandler`.
- Reconnects with exponential backoff when needed. Emits periodic heartbeat logs when idle.

### 7.3 SST1 Recorder/Player (`Recorder`)
Format (little-endian):

**Header**
```
char     magic[4]  = 'S','S','T','1'
uint32_t version   = 1
```

**Record (per frame)**
```
uint32_t width
uint32_t height
uint8_t  format      // PixelFormat
uint8_t  pad[3]      // zero
uint64_t timestamp_ns
uint64_t seq
uint32_t crc32
uint32_t data_size
uint8_t  data[data_size]
```

API:
```cpp
class FrameRecorder {
public:
  explicit FrameRecorder(const std::string& path);
  bool write(const Frame& f, std::uint64_t seq, std::uint32_t crc32);
};

class FramePlayer {
public:
  explicit FramePlayer(const std::string& path);
  bool readNext(Frame& out, std::vector<std::uint8_t>& scratch,
                std::uint64_t* out_seq = nullptr,
                std::uint64_t* out_ts_ns = nullptr,
                std::uint32_t* out_crc32 = nullptr);
};
```
Notes:
- `FramePlayer::readNext` returns a `Frame` pointing into the provided `scratch` buffer.
- Player sets `out.timestamp` to a default value; the caller controls pacing.

---

## 8. Synthetic Sources

### 8.1 `ArtimagenSource`
- Uses external ARTIMAGEN (`libartimagen`) to generate SEM-like images.
- Configures a sample (`t_gc_definition`) and image parameters (`t_std_image_def_struct`).
- For each frame, generates a new `CImage`, converts `[0..1]` doubles to `Gray8`, and returns a `Frame`.

### 8.2 `ViewportScannerSource`
```cpp
struct ViewportScannerSource::Options {
  int    tileW=512, tileH=512;
  int    gridCols=5, gridRows=5;
  double overlap=0.20;
  double driftX=5.0, driftY=2.5;    // px/s
  double jitterSigma=0.4;           // px RMS
  double flickerAmp=0.03;           // brightness amplitude
  int    masterW=4096, masterH=4096;
  unsigned seed=0;                   // 0 = auto
  std::string masterPath{};          // optional: load master from file
  std::string pattern="grid";        // "grid", "rings", "text:SEM", "checker", "noise"
};
```
Behavior:
- Generates or loads a master image (Gray8).
- Scans a viewport in a zig-zag grid order with overlap.
- Adds time-based drift, Gaussian jitter, and optional sinusoidal brightness flicker.
- Returns a `Frame` referencing an internal buffer.

---

## 9. CLI Tools

### 9.1 `simulate`
```
sem-stitch-cli simulate [--net=fast|balanced|robust] [--fps=30] [--port=50051] [--record=out.sst] [--sim=basic|scan]
# scan-specific:
  [--tilew=512] [--tileh=512] [--grid=5x5] [--overlap=0.2]
  [--driftx=0.2] [--drifty=0.1] [--jitter=0.3] [--flicker=0.03]
  [--masterw=4096] [--masterh=4096]
```
- Starts a `GrpcServer` and streams frames from either `ArtimagenSource` (basic) or `ViewportScannerSource` (scan).
- Network profiles set queue limits, heartbeat period, and compression flag.

### 9.2 `receive`
```
sem-stitch-cli receive [--net=fast|balanced|robust] [--server=localhost:50051]
                       [--latency=200] [--bufcap=256] [--drop=oldest|newest]
                       [--health=2000] [--save=mosaic.png] [--view] [--outdir=DIR]
```
- Runs a `GrpcClient`, feeds frames into `Stitcher`, and (optionally) shows a viewer.
- Uses a `JitterBuffer` with target latency and drop policy.
- Viewer hotkeys:
  - `M`: toggle mosaic vs last frame view.
  - `SPACE`: pause/resume.
  - `+`/`-`: increase/decrease target latency.
  - `S`: save mosaic.
  - `Q` or `ESC`: quit.
- Writes a run manifest JSON in `outdir` with software versions, CLI args, stats, and artifact metadata.

**Manifest fields (example outline)**
```json
{
  "mist_version": "0.1",
  "run_id": "...",
  "mode": "receive",
  "started_at": "YYYY-MM-DDTHH:MM:SSZ",
  "finished_at": "YYYY-MM-DDTHH:MM:SSZ",
  "outdir": "runs/receive_...",
  "cli": { "argv": "..." },
  "software": {
    "semstitch_version": "...",
    "git_sha": "...",
    "opencv": "..."
  },
  "input": { "grpc_address": "host:port" },
  "network": {
    "profile": "balanced",
    "latency_target_ms": 200,
    "bufcap": 256,
    "drop": "oldest"
  },
  "stats": {
    "frames_in": 0,
    "frames_out": 0,
    "drops": 0,
    "bytes_in": 0,
    "duration_s": 0.0,
    "mb_per_s": 0.0
  },
  "artifacts": [
    {
      "path": "runs/.../mosaic.png",
      "size": 0,
      "sha256": "",
      "crc32": "ABCD1234",
      "kind": "mosaic"
    }
  ]
}
```

### 9.3 `play`
Two modes:

1) **Local mosaic** (no `--port`):
```
sem-stitch-cli play --folder=DIR [--ext=png,jpg,tif] [--view] [--save=mosaic.png]
                    [--fps=30] [--loop|--repeat=N|--duration=S]
```
- Reads images as Gray8 (with normalization if needed), pushes into `Stitcher`, optional viewer, and saves a PNG mosaic.

2) **gRPC restream** (`--port >= 0`):
```
sem-stitch-cli play --folder=DIR [--fps=30] --port=50051 [--loop|--repeat=N|--duration=S]
```
- Restreams the folder as frames to a `GrpcServer` at the given port.

Notes:
- `.sst` playback is disabled in this build; use `--folder`.

---

## 10. Utilities

### 10.1 Argument Helpers (`args.hpp`)
- `argValue`, `argValueInt`, `argValueDouble`, `argHas` for simple `--key=value` parsing.

### 10.2 Viewer Helpers (`utils.hpp/.cpp`)
- `cv::Mat displayize(const cv::Mat& src8u)`: stretch grayscale to full 8-bit range if needed.
- `void save_mosaic_png(const cv::Mat& m, const std::string& path)`: save and log.

### 10.3 Jitter Buffer (`jitter_buffer.hpp`)
```cpp
class JitterBuffer {
public:
  enum class DropPolicy { Oldest, Newest };
  explicit JitterBuffer(std::size_t capacity, DropPolicy drop);

  void push(RecvFrame&& rf, std::atomic<std::uint64_t>& drops);
  bool pop_wait(RecvFrame& out, int wait_ms);
  std::size_t size() const;
};
```
- Bounded queue with drop policy, used in `receive` mode to meet target latency.

---

## 11. Coordinate and Convention Notes

- Pair match convention: `A(x,y) ~ B(x - dx, y - dy)` → `dx, dy` is the shift of **B relative to A**.
- Global solver equations use `x_j - x_i ≈ dx` and `y_j - y_i ≈ dy`.
- Tile pose is the **top-left** corner in mosaic coordinates (double, supports subpixel).
- The solver anchors tile `(0,0)` at `(0,0)`.
- The compositor shifts the mosaic so all coordinates are non-negative; it returns `origin` (negative of the applied shift) for reference.

---

## 12. Error Handling and Logging

- Many operations use `CV_Assert`/`assert`/`throw` for invalid inputs or failed external calls.
- gRPC client logs CRC mismatches, sequence gaps, duplicates, reconnects, and heartbeats.
- Server logs queue size periodically and writes heartbeats in idle periods.
- Artimagen source logs sample creation and filters verbose messages.

---

## 13. Build and Dependencies

- **Language:** C++17.
- **Required:** OpenCV (core, imgproc, highgui for viewer), gRPC + protobuf, zlib.
- **Optional:** OpenSSL (for SHA-256 in helper utilities).
- **Threading:** standard C++ threads; background threads for gRPC, viewer, health logging, and consumer loops.

> Note: The repository should provide build scripts (e.g., CMake). If not present, integrate the listed dependencies and compile all sources in `core`, `align`, `compose`, `io`, and `cli`.

---

## 14. Extensibility

- **Backends:** Implement `IBackend` for CUDA/OpenCL or other methods; register through `makeBackend`.
- **Alignment:** Replace or extend the local refinement or the global solver (e.g., add rotation/shear).
- **Compositor:** Add feathering strategies or multi-band blending.
- **I/O:** Extend the protobuf schema, add compression, or add new streaming transports.

---

## 15. Limitations

- Only translation is estimated; no rotation/scale in the default pipeline.
- `Gray8` is the primary tested format.
- CUDA/OpenCL backends are not implemented.
- `.sst` playback mode is disabled in the current CLI build; use folders.

---

## 16. Example Pipelines

### 16.1 Live streaming over gRPC
```
# Terminal A (server/source)
sem-stitch-cli simulate --net=balanced --fps=15 --port=50051

# Terminal B (client/receiver with viewer)
sem-stitch-cli receive --server=localhost:50051 --view --save=mosaic.png
```

### 16.2 Local folder to mosaic
```
sem-stitch-cli play --folder=./images --fps=30 --view --save=mosaic.png
```

### 16.3 Synthetic “scan” source to server
```
sem-stitch-cli simulate --sim=scan --grid=8x6 --tilew=512 --tileh=512                         --overlap=0.2 --fps=10 --port=50051
```
