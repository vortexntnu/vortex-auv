#ifndef POSE_FILTERING__LIB__HUNGARIAN_HPP_
#define POSE_FILTERING__LIB__HUNGARIAN_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>

namespace vortex::filtering {

/**
 * @brief Minimum-cost assignment of rows to columns (Hungarian algorithm,
 * O(n^2 m)). Exact and deterministic.
 *
 * @param cost rows x cols cost matrix with rows <= cols. Forbidden pairs get
 * a large finite cost (not infinity).
 * @return For each row, the assigned column.
 */
std::vector<int> solve_assignment(const Eigen::MatrixXd& cost);

/**
 * @brief Global nearest neighbour association between tracks and
 * measurements.
 *
 * Every track gets at most one measurement and every measurement at most one
 * track. A pair is allowed when `allowed(t, m)`; its cost is `cost(t, m)`,
 * normally the squared Mahalanobis distance. Leaving a track and a
 * measurement unpaired costs `unpaired_cost` together, so an allowed pair is
 * only made when it is cheaper than that (with a gate on d^2 <= gamma^2,
 * pass gamma^2 and every allowed pair is taken if possible).
 *
 * @param cost num_tracks x num_measurements.
 * @param allowed Same size, true where the pair passed the gate.
 * @param unpaired_cost Cost of leaving one track and one measurement
 * unpaired.
 * @return For each track, the index of its measurement, or -1.
 */
std::vector<int> associate_gnn(
    const Eigen::MatrixXd& cost,
    const Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>& allowed,
    double unpaired_cost);

}  // namespace vortex::filtering

#endif  // POSE_FILTERING__LIB__HUNGARIAN_HPP_
