#include "landmark_server/measure.hpp"

#include <fmt/format.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>

namespace vortex::landmark_server {

namespace {

using LS = vortex_msgs::msg::LandmarkSubtype;
using LT = vortex_msgs::msg::LandmarkType;

// shortcut: white pipes are ~1.6 m from their red one and 2.1+ m from the
// next row's on the sim course; make it a setting if the real course differs.
constexpr double kMaxWhiteToRedM = 1.8;

std::vector<Landmark> clusters_of(const std::vector<const Hit*>& hits,
                                  const MeasureSettings& s) {
    std::vector<Eigen::Vector3d> points;
    for (const Hit* h : hits) {
        points.push_back(h->position);
    }
    std::vector<Landmark> out;
    for (const auto& members : cluster(points, s.cluster_radius_m,
                                       static_cast<std::size_t>(s.min_hits))) {
        std::vector<const Hit*> in;
        for (const std::size_t i : members) {
            in.push_back(hits[i]);
        }
        out.push_back(summarise(in));
    }
    return out;
}

double yaw_from_vehicle(const Pose& T_odom_base, const Eigen::Vector3d& p) {
    const Eigen::Vector3d d = p - T_odom_base.translation();
    return std::atan2(d.y(), d.x());
}

Measurement failed(std::string message, int hits) {
    Measurement m;
    m.hits = hits;
    m.message = std::move(message);
    return m;
}

}  // namespace

Measurement measure_object(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s) {
    std::vector<const Hit*> all;
    for (const Hit& h : hits) {
        all.push_back(&h);
    }
    const int n = static_cast<int>(hits.size());
    const auto clusters = clusters_of(all, s);
    if (clusters.empty()) {
        return failed(fmt::format("{} hits, no {} within {:.2f} m", n,
                                  s.min_hits, s.cluster_radius_m),
                      n);
    }
    const Landmark& l = clusters.front();
    Measurement m;
    m.hits = l.n_obs;
    m.spread_m = l.spread_m;
    if (l.spread_m > s.max_spread_m) {
        m.message = fmt::format("spread {:.2f} m > {:.2f} m", l.spread_m,
                                s.max_spread_m);
        return m;
    }
    const double yaw =
        l.yaw_known ? l.yaw : yaw_from_vehicle(T_odom_base, l.position);
    m.pose = yaw_pose(yaw, l.position);
    m.ok = true;
    m.message = fmt::format("{} hits, spread {:.2f} m", m.hits, m.spread_m);
    return m;
}

Measurement measure_slalom_row(const std::vector<Hit>& hits,
                               const Pose& T_odom_base,
                               const MeasureSettings& s) {
    std::vector<const Hit*> red_hits;
    std::vector<const Hit*> white_hits;
    for (const Hit& h : hits) {
        if (h.cls->type != LT::SLALOM_PIPE) {
            continue;
        }
        (h.cls->subtype == LS::SLALOM_PIPE_RED ? red_hits : white_hits)
            .push_back(&h);
    }
    const int n = static_cast<int>(red_hits.size() + white_hits.size());
    const auto reds = clusters_of(red_hits, s);
    const auto whites = clusters_of(white_hits, s);

    // The nearest red pipe in front of the vehicle.
    const Pose T_base_odom = T_odom_base.inverse();
    const Landmark* red = nullptr;
    double best = std::numeric_limits<double>::max();
    for (const Landmark& r : reds) {
        const Eigen::Vector3d b = T_base_odom * r.position;
        if (b.x() > 0.0 && b.head<2>().norm() < best) {
            best = b.head<2>().norm();
            red = &r;
        }
    }
    if (!red) {
        return failed(fmt::format("{} pipe hits, no red pipe in front", n), n);
    }

    Measurement m;
    m.hits = red->n_obs;
    m.spread_m = red->spread_m;
    std::vector<Eigen::Vector2d> row{red->position.head<2>()};
    for (const Landmark& w : whites) {
        const Landmark* nearest = nullptr;
        double d_best = kMaxWhiteToRedM;
        for (const Landmark& r : reds) {
            const double d = (w.position - r.position).head<2>().norm();
            if (d < d_best) {
                d_best = d;
                nearest = &r;
            }
        }
        if (nearest == red) {
            row.push_back(w.position.head<2>());
            m.hits += w.n_obs;
            m.spread_m = std::max(m.spread_m, w.spread_m);
        }
    }
    if (m.spread_m > s.max_spread_m) {
        m.message = fmt::format("spread {:.2f} m > {:.2f} m", m.spread_m,
                                s.max_spread_m);
        return m;
    }

    const Eigen::Vector2d to_red =
        (red->position - T_odom_base.translation()).head<2>();
    Eigen::Vector2d x = to_red;
    if (row.size() > 1) {
        // +X across the line of the pipes, away from the vehicle.
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (const auto& p : row) {
            mean += p;
        }
        mean /= static_cast<double>(row.size());
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto& p : row) {
            cov += (p - mean) * (p - mean).transpose();
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
        const Eigen::Vector2d along = eig.eigenvectors().col(1);
        x = Eigen::Vector2d(along.y(), -along.x());
        if (x.dot(to_red) < 0.0) {
            x = -x;
        }
    }
    m.pose = yaw_pose(std::atan2(x.y(), x.x()), red->position);
    m.ok = true;
    m.message = fmt::format("row with {} pipes, {} hits, spread {:.2f} m",
                            row.size(), m.hits, m.spread_m);
    return m;
}

Measurement measure_slalom(const std::vector<Hit>& hits,
                           const Pose& T_odom_base,
                           const MeasureSettings& s) {
    std::vector<const Hit*> red_hits;
    std::vector<const Hit*> white_hits;
    for (const Hit& h : hits) {
        if (h.cls->type != LT::SLALOM_PIPE) {
            continue;
        }
        (h.cls->subtype == LS::SLALOM_PIPE_RED ? red_hits : white_hits)
            .push_back(&h);
    }
    const int n = static_cast<int>(red_hits.size() + white_hits.size());
    struct Pipe {
        Eigen::Vector3d p;
        bool red;
    };
    std::vector<Pipe> pipes;
    Measurement m;
    for (const auto& [group, red] :
         {std::pair{&red_hits, true}, std::pair{&white_hits, false}}) {
        for (const Landmark& l : clusters_of(*group, s)) {
            pipes.push_back({l.position, red});
            m.hits += l.n_obs;
            m.spread_m = std::max(m.spread_m, l.spread_m);
        }
    }
    if (std::none_of(pipes.begin(), pipes.end(),
                     [](const Pipe& q) { return q.red; })) {
        return failed(fmt::format("{} pipe hits, no red pipe", n), n);
    }
    if (m.spread_m > s.max_spread_m) {
        m.message = fmt::format("spread {:.2f} m > {:.2f} m", m.spread_m,
                                s.max_spread_m);
        return m;
    }

    // Heading through the rows: the one where the pipes fall best on lines
    // spacing_m apart, a little in favour of where the vehicle looks.
    const double D = s.slalom_spacing_m;
    const double heading = yaw_of(T_odom_base.rotation());
    auto along = [](const Eigen::Vector3d& p, double th) {
        return p.x() * std::cos(th) + p.y() * std::sin(th);
    };
    auto row_offset = [&](double th, double* cost) {
        double best_cost = std::numeric_limits<double>::max();
        double best_a0 = 0.0;
        for (const Pipe& ref : pipes) {
            double a0 = along(ref.p, th);
            for (int it = 0; it < 2; ++it) {
                double c = 0.0, mean = 0.0;
                for (const Pipe& q : pipes) {
                    const double a = along(q.p, th) - a0;
                    const double r = a - D * std::round(a / D);
                    c += r * r;
                    mean += r;
                }
                a0 += mean / static_cast<double>(pipes.size());
                if (it == 1 && c < best_cost) {
                    best_cost = c;
                    best_a0 = a0;
                }
            }
        }
        *cost = best_cost;
        return best_a0;
    };
    double theta = heading;
    double a0 = 0.0;
    double best = std::numeric_limits<double>::max();
    for (int deg = -90; deg <= 90; ++deg) {
        const double th = wrap(heading + deg * M_PI / 180.0);
        double c = 0.0;
        const double a = row_offset(th, &c);
        c += 1e-3 * std::pow(deg * M_PI / 180.0, 2);
        if (c < best) {
            best = c;
            theta = th;
            a0 = a;
        }
    }

    // Rows from the first one with a pipe in front of the vehicle.
    const double a_vehicle = along(T_odom_base.translation(), theta);
    std::optional<int> first;
    for (const Pipe& q : pipes) {
        const int k =
            static_cast<int>(std::lround((along(q.p, theta) - a0) / D));
        if (a0 + k * D > a_vehicle && (!first || k < *first)) {
            first = k;
        }
    }
    if (!first) {
        return failed(fmt::format("{} pipe hits, no row in front", n), n);
    }
    const Eigen::Vector2d u(-std::sin(theta), std::cos(theta));
    double z = 0.0;
    for (const Pipe& q : pipes) {
        z += q.p.z() / static_cast<double>(pipes.size());
    }
    // Sideways: the row's red pipe, else the middle of its pipes, else the
    // row before.
    std::optional<double> lateral;
    for (const Pipe& q : pipes) {
        if (q.red && std::lround((along(q.p, theta) - a0) / D) >= *first) {
            if (!lateral) {
                lateral = q.p.head<2>().dot(u);
            }
        }
    }
    int seen = 0;
    for (int k = 0; k < s.slalom_rows; ++k) {
        const int row = *first + k;
        std::optional<double> red_l;
        double sum = 0.0;
        int count = 0;
        for (const Pipe& q : pipes) {
            if (std::lround((along(q.p, theta) - a0) / D) != row) {
                continue;
            }
            const double l = q.p.head<2>().dot(u);
            if (q.red) {
                red_l = l;
            }
            sum += l;
            ++count;
        }
        if (red_l) {
            lateral = red_l;
        } else if (count > 1) {
            lateral = sum / count;
        }
        seen += count > 0;
        const double a = a0 + row * D;
        const Eigen::Vector2d xy =
            a * Eigen::Vector2d(std::cos(theta), std::sin(theta)) +
            lateral.value_or(0.0) * u;
        m.rows.push_back(yaw_pose(theta, Eigen::Vector3d(xy.x(), xy.y(), z)));
    }
    m.pose = m.rows.front();
    m.ok = true;
    m.message =
        fmt::format("{} of {} rows seen, {} pipes, {} hits, spread {:.2f} m",
                    seen, s.slalom_rows, pipes.size(), m.hits, m.spread_m);
    return m;
}

}  // namespace vortex::landmark_server
