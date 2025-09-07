#include "semstitch/align/GlobalSolver.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>

/*
 Global translation solver for a grid of image tiles.

 Input:
   - AlignResult matches: pairwise edges with dx, dy, quality scores.
   - rows, cols: grid size.
   - SolveOptions opt: thresholds and IRLS parameters.

 Output:
   - SolveResult: per-tile (x, y) positions, statistics, and a short log.

 Method:
   1) Build equations from edges: (-x_i + x_j) = dx and (-y_i + y_j) = dy.
   2) Use IRLS with Huber weights and edge confidence.
   3) Softly anchor node 0 at (0, 0) to fix gauge freedom.
*/

namespace semstitch {

// Flatten (row, col) index to 1D index: r*cols + c
static int idxRC(int r, int c, int cols) { return r*cols + c; }

// Initialize tile positions by median step in X and Y (robust to outliers).
std::vector<cv::Point2d> initializeByMedians(const AlignResult& m, int rows, int cols)
{
    std::vector<double> rights, downs;
    rights.reserve(m.edges.size());
    downs.reserve(m.edges.size());

    for (const auto& e : m.edges) {
        if (!e.ok) continue;
        if (e.direction == "right") rights.push_back(e.fine.dx);
        else if (e.direction == "down") downs.push_back(e.fine.dy);
    }
    auto median = [](std::vector<double>& v)->double{
        if (v.empty()) return 0.0;
        std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
        return v[v.size()/2];
    };
    double stepX = median(rights);
    double stepY = median(downs);

    std::vector<cv::Point2d> P(rows*cols, {0.0,0.0});
    for (int r=0; r<rows; ++r)
        for (int c=0; c<cols; ++c)
            P[idxRC(r,c,cols)] = cv::Point2d(c*stepX, r*stepY);
    // Anchor (0,0) is already (0,0)
    return P;
}

// Robust Huber weight for IRLS: 1 if s <= delta; otherwise delta / s.
static inline double huber_weight(double s, double delta) {
    // s = ||residual||
    if (s <= 1e-12) return 1.0;
    return (s <= delta) ? 1.0 : (delta / s);
}

// Convert ZNCC score to confidence [0..1] with a power curve.
// If zncc < min_use, confidence is 0 (edge is ignored).
static inline double zncc_to_conf(double zncc, double powK, double min_use) {
    if (zncc < min_use) return 0.0;
    double x = std::max(0.0, zncc);
    return std::pow(x, powK);
}

SolveResult solveGlobalTranslation(const AlignResult& matches,
                                   int rows, int cols,
                                   const SolveOptions& opt)
{
    const int N = rows*cols;
    SolveResult sr;
    if (N <= 0) { sr.report = "empty grid"; return sr; }

    // Collect usable edges (filter by thresholds)
    struct EdgeRow {
        int i, j;           // node indices
        double dx, dy;      // measurement: (x_j - x_i, y_j - y_i)
        double conf;        // confidence in [0..1]
    };
    std::vector<EdgeRow> E; E.reserve(matches.edges.size());

    for (const auto& e : matches.edges) {
        if (!e.ok) continue;
        if (e.coarse.score < opt.phase_resp_min) continue;
        double conf = zncc_to_conf(e.fine.score, opt.zncc_pow, opt.min_use_zncc);
        if (conf <= 0.0) continue;

        int i = idxRC(e.r1, e.c1, cols);
        int j = idxRC(e.r2, e.c2, cols);
        // By definition: A(x,y) ~ B(x - dx, y - dy)
        //  ⇒ x_j - x_i ≈ dx; y_j - y_i ≈ dy
        E.push_back({i, j, e.fine.dx, e.fine.dy, conf});
    }

    // Initial poses
    std::vector<cv::Point2d> P = initializeByMedians(matches, rows, cols);

    // IRLS (we solve in a blocked layout: [x_0..x_{N-1}, y_0..y_{N-1}])
    const int DIM = 2*N;
    auto pack = [&](int k, bool isX){ return isX ? k : (k + N); };

    auto build_and_solve = [&](const std::vector<double>& wHuber, std::vector<double>& sol_out)->bool {
        cv::Mat_<double> Nmat = cv::Mat::zeros(DIM, DIM, CV_64F); // A^T W A
        cv::Mat_<double> rhs  = cv::Mat::zeros(DIM, 1,   CV_64F); // A^T W b

        // For each edge: (-x_i + x_j) = dx; (-y_i + y_j) = dy; weight = conf * wHuber
        for (size_t k=0; k<E.size(); ++k) {
            const auto& er = E[k];
            double w = er.conf * wHuber[k];
            if (w <= 0.0) continue;

            int ix = pack(er.i, true),  jx = pack(er.j, true);
            int iy = pack(er.i, false), jy = pack(er.j, false);

            // X-equation
            Nmat(ix, ix) += w;  Nmat(jx, jx) += w;
            Nmat(ix, jx) -= w;  Nmat(jx, ix) -= w;
            rhs(ix,0)   += -w * er.dx;
            rhs(jx,0)   += +w * er.dx;

            // Y-equation
            Nmat(iy, iy) += w;  Nmat(jy, jy) += w;
            Nmat(iy, jy) -= w;  Nmat(jy, iy) -= w;
            rhs(iy,0)   += -w * er.dy;
            rhs(jy,0)   += +w * er.dy;
        }

        // Soft anchor for node 0: x0 = 0, y0 = 0 (large weight)
        Nmat(pack(0,true),  pack(0,true))  += opt.anchor_weight;
        Nmat(pack(0,false), pack(0,false)) += opt.anchor_weight;
        // rhs stays 0

        // Solve normal equations; try several decompositions
        cv::Mat_<double> sol;
        bool ok = cv::solve(Nmat, rhs, sol, cv::DECOMP_CHOLESKY);
        if (!ok) ok = cv::solve(Nmat, rhs, sol, cv::DECOMP_EIG);
        if (!ok) ok = cv::solve(Nmat, rhs, sol, cv::DECOMP_SVD);
        if (!ok) return false;

        sol_out.assign(DIM, 0.0);
        for (int i=0;i<DIM;++i) sol_out[i] = sol(i,0);
        return true;
    };

    // Main IRLS loop
    int it=0;
    std::vector<double> wHuber(E.size(), 1.0);
    std::ostringstream rep;
    for (; it<opt.max_iters; ++it) {
        // 1) Build the system and solve for increment δ.
        //    (This is equivalent to solving A^T W A δ = A^T W (b - A P).)
        std::vector<double> delta;
        if (!build_and_solve(wHuber, delta)) { rep << "linear solve failed\n"; break; }

        // 2) Update poses
        double maxAbs = 0.0;
        for (int k=0;k<N;++k) {
            P[k].x += delta[pack(k,true)];
            P[k].y += delta[pack(k,false)];
            maxAbs = std::max(maxAbs, std::abs(delta[pack(k,true)]));
            maxAbs = std::max(maxAbs, std::abs(delta[pack(k,false)]));
        }

        // 3) Recompute robust weights from current residuals
        double rmse_acc=0.0; int cnt=0;
        for (size_t k=0; k<E.size(); ++k) {
            const auto& er = E[k];
            cv::Point2d r(
                (P[er.j].x - P[er.i].x) - er.dx,
                (P[er.j].y - P[er.i].y) - er.dy
            );
            double s = std::sqrt(r.x*r.x + r.y*r.y);
            rmse_acc += s*s; ++cnt;

            double w = huber_weight(s, opt.huber_delta_px);
            // Low-confidence edges are already down-weighted by er.conf; here we apply only the robust factor
            wHuber[k] = w;
        }
        double rmse = (cnt>0) ? std::sqrt(rmse_acc / cnt) : 0.0;

        rep << "[iter " << it << "] max|δ|=" << maxAbs << " rmse=" << rmse << "\n";
        if (maxAbs < opt.stop_eps) { ++it; break; } // count this iteration as completed
    }

    // Final metrics
    int inl=0, outl=0; double rmse_acc=0.0; int cnt=0;
    for (const auto& er : E) {
        cv::Point2d r(
            (P[er.j].x - P[er.i].x) - er.dx,
            (P[er.j].y - P[er.i].y) - er.dy
        );
        double s = std::sqrt(r.x*r.x + r.y*r.y);
        rmse_acc += s*s; ++cnt;
        if (s <= 2.0*opt.huber_delta_px) ++inl; else ++outl;
    }

    sr.poses      = std::move(P);
    sr.iters      = it;
    sr.rmse_px    = (cnt>0) ? std::sqrt(rmse_acc / cnt) : 0.0;
    sr.used_edges = (int)E.size();
    sr.inliers    = inl;
    sr.outliers   = outl;
    sr.report     = rep.str();
    return sr;
}

} // namespace semstitch
