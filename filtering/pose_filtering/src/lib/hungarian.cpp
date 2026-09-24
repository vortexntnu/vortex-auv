#include "pose_filtering/lib/hungarian.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vortex::filtering {

std::vector<int> solve_assignment(const Eigen::MatrixXd& cost) {
    const int n = static_cast<int>(cost.rows());
    const int m = static_cast<int>(cost.cols());
    if (n == 0) {
        return {};
    }
    if (n > m) {
        throw std::invalid_argument("solve_assignment: rows must be <= cols");
    }

    // Shortest augmenting path with potentials, 1-indexed (row/col 0 is a
    // virtual start).
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, inf);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = inf;
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) {
                    continue;
                }
                const double cur = cost(i0 - 1, j - 1) - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> row_to_col(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) {
            row_to_col[p[j] - 1] = j - 1;
        }
    }
    return row_to_col;
}

std::vector<int> associate_gnn(
    const Eigen::MatrixXd& cost,
    const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>& allowed,
    double unpaired_cost) {
    const int n_t = static_cast<int>(cost.rows());
    const int n_m = static_cast<int>(cost.cols());
    std::vector<int> out(n_t, -1);
    if (n_t == 0 || n_m == 0) {
        return out;
    }

    // Square (T + M) x (M + T) matrix:
    //   [ pair costs      | track t unpaired (diagonal) ]
    //   [ meas m unpaired | 0                           ]
    // Forbidden entries get a cost larger than any full solution.
    const double half = 0.5 * unpaired_cost;
    const double forbidden =
        1e6 +
        2.0 * (n_t + n_m) *
            (std::abs(unpaired_cost) +
             (allowed.any()
                  ? (allowed.cast<double>() * cost.array().abs()).maxCoeff()
                  : 0.0));
    const int n = n_t + n_m;
    Eigen::MatrixXd c = Eigen::MatrixXd::Constant(n, n, forbidden);
    for (int t = 0; t < n_t; ++t) {
        for (int m = 0; m < n_m; ++m) {
            if (allowed(t, m)) {
                c(t, m) = cost(t, m);
            }
        }
        c(t, n_m + t) = half;
    }
    for (int m = 0; m < n_m; ++m) {
        c(n_t + m, m) = half;
    }
    c.bottomRightCorner(n_m, n_t).setZero();

    const auto assignment = solve_assignment(c);
    for (int t = 0; t < n_t; ++t) {
        const int col = assignment[t];
        if (col < n_m && allowed(t, col)) {
            out[t] = col;
        }
    }
    return out;
}

}  // namespace vortex::filtering
