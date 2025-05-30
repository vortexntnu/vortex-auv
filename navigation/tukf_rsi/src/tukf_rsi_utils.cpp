
#include "tukf_rsi_utils.hpp"
#include <cmath>

Eigen::Quaterniond quaternionMean(
    const std::vector<Eigen::Quaterniond>& quats,
    double tol,
    int maxIter
) {
    Eigen::Quaterniond mean_q = quats.front();
    int n = int(quats.size());

    for (int iter = 0; iter < maxIter; ++iter) {
        Eigen::Vector3d errAvg = Eigen::Vector3d::Zero();

        for (const auto& q : quats) {

            Eigen::Quaterniond e = q * mean_q.conjugate();

            double w = std::clamp(e.w(), -1.0, 1.0);
            double angle = 2 * std::acos(w);

            Eigen::Vector3d axis;

            if (std::abs(angle) < tol) {
                axis.setZero();
            } else {
                axis = (angle / std::sin(angle / 2.0)) * e.vec();
            }
            errAvg += axis;
        }
        errAvg /= double(n);

        if (errAvg.norm() < tol) break;
        double errNorm = errAvg.norm();
        Eigen::Quaterniond dq

        if (errNorm > tol) {
            dq.w() = std::cos(errNorm/2.0);
            dq.vec() = std::sin(errNorm/2.0) * (errAvg/errNorm);
        } else {
            dq = Eigen::Quaterniond::Identity();
        }
        mean_q = dq * mean_q;
        mean_q.normalize();
    }
    return mean_q;
}

Eigen::Vector37d mean_set(
    const std::vector<AUVState>& setPoints,
    double tol,
    int maxIter
) {
    int n = int(setPoints.size());

    Eigen::Vector3d posAvg = Eigen::Vector3d::Zero();
    Eigen::Vector3d velAvg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angVelAvg = Eigen::Vector3d::Zero();
    Eigen::Vector9d inAvg = Eigen::Vector9d::Zero();
    Eigen::Vector6d amAvg = Eigen::Vector6d::Zero();
    Eigen::Vector6d dAvg = Eigen::Vector6d::Zero();
    Eigen::Vector4d gAvg = Eigen::Vector4d::Zero();

    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(n);

    for (const auto& s : setPoints) {
        posAvg += s.position;
        velAvg += s.velocity;
        angVelAvg += s.angular_velocity;
        inAvg += s.inertia;
        amAvg += s.added_mass;
        dAvg += s.damping;
        gAvg += s.g_eta;
        quats.push_back(s.orientation);
    }
    posAvg /= double(n);
    velAvg /= double(n);
    angVelAvg /= double(n);
    inAvg /= double(n);
    amAvg /= double(n);
    dAvg /= double(n);
    gAvg /= double(n);
    Eigen::Quaterniond qMean = quaternion_mean(quats, tol, maxIter);

    AUVState meanState;
    meanState.position = posAvg;
    meanState.orientation = qMean;
    meanState.velocity = velAvg;
    meanState.angular_velocity = angVelAvg;
    meanState.inertia = inAvg;
    meanState.added_mass = amAvg;
    meanState.damping = dAvg;
    meanState.g_eta = gAvg;

    return meanState.asVector();
}

Eigen::Vector3d mean_measurement(const std::vector<MeasModel>& setPoints) {
    Eigen::Vector3d avg = Eigen::Vector3d::Zero();
    for (const auto& m : setPoints) avg += m.measurement;
    return avg / double(setPoints.size());
}

Eigen::Matrix37d covariance_set(
    const std::vector<AUVState>& setPoints,
    const Eigen::Vector37d& meanVec,
    double tol
) {
    int n = int(setPoints.size());
    AUVState meanState;
    meanState.fillStates(meanVec);
    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(n);
    
    for (const auto& s : setPoints) quats.push_back(s.orientation);
    meanState.orientation = quaternion_mean(quats, tol);

    Eigen::Matrix37d cov = Eigen::Matrix37d::Zero();
    for (const auto& s : setPoints) {
        Eigen::Vector37d d = Eigen::Vector37d::Zero();

        d.segment<3>(0) = s.position - meanState.position;

        Eigen::Quaterniond e = s.orientation * meanState.orientation.conjugate();
        double w = std::clamp(e.w(), -1.0, 1.0);
        double angle = 2 * std::acos(w);


        Eigen::Vector3d err;
        if (std::abs(angle) < tol) err.setZero();
        else err = (angle / std::sin(angle/2.0)) * e.vec();

        d.segment<3>(3) = err;
  
        d.segment<3>(6) = s.velocity - meanState.velocity;
        d.segment<3>(9) = s.angular_velocity - meanState.angular_velocity;
        d.segment<9>(12) = s.inertia - meanState.inertia;
        d.segment<6>(21) = s.added_mass - meanState.added_mass;
        d.segment<6>(27) = s.damping - meanState.damping;
        d.segment<4>(33) = s.g_eta - meanState.g_eta;
        cov += d * d.transpose();
    }
    return cov / double(n);
}

Eigen::Matrix3d covariance_measurement(
    const std::vector<MeasModel>& setPoints,
    const Eigen::Vector3d& mean
) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& m : setPoints) {
        Eigen::Vector3d d = m.measurement - mean;
        cov += d * d.transpose();
    }
    return cov / double(setPoints.size());
}

Eigen::Matrix<double,37,3> cross_covariance(
    const std::vector<AUVState>& setY,
    const Eigen::Vector37d& meanY,
    const std::vector<MeasModel>& setZ,
    const Eigen::Vector3d& meanZ,
    double tol
) {
    int n = int(setY.size());
    AUVState meanState;
    meanState.fill_states(meanY);
    std::vector<Eigen::Quaterniond> quats;
    quats.reserve(n);
    for (const auto& s : setY) quats.push_back(s.orientation);
    meanState.orientation = quaternion_cean(quats, tol);

    Eigen::Matrix<double,37,3> cov = Eigen::Matrix<double,37,3>::Zero();
    for (size_t i = 0; i < setY.size(); ++i) {
        const auto& s = setY[i];

        Eigen::Vector37d dY = Eigen::Vector37d::Zero();
        dY.segment<3>(0) = s.position - meanState.position;
        Eigen::Quaterniond e = s.orientation * meanState.orientation.conjugate();
        double w = std::clamp(e.w(), -1.0, 1.0);
        double angle = 2 * std::acos(w);
        Eigen::Vector3d err;
        if (std::abs(angle) < tol) err.setZero();
        else err = (angle / std::sin(angle/2.0)) * e.vec();
        dY.segment<3>(3) = err;
        dY.segment<3>(6) = s.velocity - meanState.velocity;
        dY.segment<3>(9) = s.angular_velocity - meanState.angular_velocity;
        dY.segment<9>(12) = s.inertia - meanState.inertia;
        dY.segment<6>(21) = s.added_mass - meanState.added_mass;
        dY.segment<6>(27) = s.damping - meanState.damping;
        dY.segment<4>(33) = s.g_eta - meanState.g_eta;
 
        Eigen::Vector3d dZ = setZ[i].measurement - meanZ;
        cov += dY * dZ.transpose();
    }
    return cov / double(n);
}