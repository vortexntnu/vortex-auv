#ifndef AUV_TYPEDEFS_HPP
#define AUV_TYPEDEFS_HPP

#include <functional>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace Eigen {
    typedef Matrix<double, 37, 1> Vector37d;
    typedef Matrix<double, 37, 37> Matrix37d;
    typedef Matrix<double, 25, 1> Vector25d;
    typedef Matrix<double, 12, 1> Vector12d;
    typedef Matrix<double,  9, 1> Vector9d;
    typedef Matrix<double,  6, 1> Vector6d;
    typedef Matrix<double,  4, 1> Vector4d;
    typedef Matrix<double,  3, 12> Matrix3x12d;
    typedef Matrix<double,  3, 3> Matrix3d;
    typedef Matrix<double,  3, 37> Matrix3x37d;
    typedef Matrix<double,  37, 74> Matrix37x74d;
    typedef Matrix<double,  37, 3> Matrix37x3d;
}

struct AUVState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector9d inertia = Eigen::Vector9d::Zero();
    Eigen::Vector6d added_mass = Eigen::Vector6d::Zero();
    Eigen::Vector6d damping = Eigen::Vector6d::Zero();
    Eigen::Vector4d g_eta = Eigen::Vector4d::Zero();
    Eigen::Matrix37d covariance = Eigen::Matrix37d::Zero();

    Eigen::Vector3d error = Eigen::Vector3d::Zero();

    AUVState() = default;

    Eigen::Vector12d dynamic_part() const {
        Eigen::Vector12d x;
        x << position,
             orientation.vec(),
             velocity,
             angular_velocity;
        return x;
    }

    Eigen::Vector25d okid_part() const {
        Eigen::Vector25d x;
        x << inertia,
             added_mass,
             damping,
             g_eta;
        return x;
    }

    Eigen::Vector37d as_vector() const {
        Eigen::Vector37d x;
        x << dynamic_part(),
             okid_part();
        return x;
    }

    AUVState operator+(const AUVState& other) const {
        AUVState result;
        result.position = position + other.position;
        result.orientation = orientation * other.orientation;
        result.velocity = velocity + other.velocity;
        result.angular_velocity = angular_velocity + other.angular_velocity;
        result.inertia = inertia + other.inertia;
        result.added_mass = added_mass + other.added_mass;
        result.damping = damping + other.damping;
        result.g_eta = g_eta + other.g_eta;
        return result;
    }

    AUVState operator-(const AUVState& other) const {
        AUVState result;
        result.position = position - other.position;
        result.orientation = orientation * other.orientation.inverse();
        result.velocity = velocity - other.velocity;
        result.angular_velocity = angular_velocity - other.angular_velocity;
        result.inertia = inertia - other.inertia;
        result.added_mass = added_mass - other.added_mass;
        result.damping = damping - other.damping;
        result.g_eta = g_eta - other.g_eta;
        return result;
    }

    void fill_states(const Eigen::Vector37d& x) {
        position = x.segment<3>(0);
        Eigen::Vector3d ori_vec = x.segment<3>(3);
        orientation = Eigen::Quaterniond(1, ori_vec.x(), ori_vec.y(), ori_vec.z()).normalized();
        velocity = x.segment<3>(6);
        angular_velocity = x.segment<3>(9);
        inertia = x.segment<9>(12);
        added_mass = x.segment<6>(21);
        damping = x.segment<6>(27);
        g_eta = x.segment<4>(33);
    }
};

struct MeasModel {
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance  = Eigen::Matrix3d::Zero();
    std::function<MeasModel(const AUVState&)> H;

    MeasModel()
        : H(default_h)
    {}

    MeasModel(const Eigen::Vector3d& meas,
              const Eigen::Matrix3d& cov,
              std::function<MeasModel(const AUVState&)> Hfunc = default_h)
        : measurement(meas), covariance(cov), H(std::move(Hfunc))
    {}

    static MeasModel default_h(const AUVState& state) {
        MeasModel z;
        Eigen::Matrix3x12d Hmat = Eigen::Matrix3x12d::Zero();
        Hmat.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
        z.measurement = Hmat * state.dynamic_part();
        return z;
    }

    MeasModel operator+(const MeasModel& other) const {
        MeasModel r;
        r.measurement = measurement + other.measurement;
        return r;
    }

    MeasModel operator-(const MeasModel& other) const {
        MeasModel r;
        r.measurement = measurement - other.measurement;
        return r;
    }

    friend MeasModel operator*(double scalar, const MeasModel& m) {
        MeasModel r;
        r.measurement = scalar * m.measurement;
        return r;
    }
};

#endif // AUV_TYPEDEFS_HPP