#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>



namespace eskf
{
enum NominalStateMembers  //(16x1)
{
  StateMemberX = 0,
  StateMemberY = 1,
  StateMemberZ = 2,
  StateMemberVx = 3,
  StateMemberVy = 4,
  StateMemberVz = 5,
  StateMemberQw = 6,
  StateMemberQx = 7,
  StateMemberQy = 8,
  StateMemberQz = 9,
  StateMemberAccBiasX = 10,
  StateMemberAccBiasY = 11,
  StateMemberAccBiasZ = 12,
  StateMemberGyroBiasX = 13,
  StateMemberGyroBiasY = 14,
  StateMemberGyroBiasZ = 15,
  StateMemberGravityX = 16,
  StateMemberGravityY = 17,
  StateMemberGravityZ = 18
};

constexpr int NOMINAL_STATE_SIZE{ 19 };  // 16
constexpr int ERROR_STATE_SIZE{ 18 };
constexpr int NOMINAL_POSITION_STATE_OFFSET = StateMemberX;
constexpr int NOMINAL_VELOCITY_STATE_OFFSET = StateMemberVx;
constexpr int NOMINAL_QUATERNION_STATE_OFFSET = StateMemberQw;
constexpr int NOMINAL_ACC_BIAS_STATE_OFFSET = StateMemberAccBiasX;
constexpr int NOMINAL_GYRO_BIAS_STATE_OFFSET = StateMemberGyroBiasX;
constexpr int NOMINAL_GRAVITY_STATE_OFFSET = StateMemberGravityX;
constexpr int NOMINAL_POSITION_STATE_SIZE{ 3 };
constexpr int NOMINAL_VELOCITY_STATE_SIZE{ 3 };
constexpr int NOMINAL_QUATERNION_STATE_SIZE{ 4 };
constexpr int NOMINAL_ACC_BIAS_SIZE{ 3 };
constexpr int NOMINAL_GYRO_BIAS_SIZE{ 3 };
constexpr int NOMINAL_GRAVITY_SIZE{ 3 };
constexpr double GRAVITY{ 9.80665 };
constexpr int DEFAULT_IMU_RATE{ 125 };

struct AdandGQGD
{
  Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE> Ad;    //(15x15)
  Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE> GQGD;  //(15x15)
};

struct StatesAndErrorCovariance
{
  Eigen::Matrix<double, NOMINAL_STATE_SIZE, 1> X{ Eigen::Matrix<double, NOMINAL_STATE_SIZE, 1>::Zero() };
  Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE> P{ Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE>::Zero() };
};

struct InnovationPressureStates
{
  // Matrix<double,1,1> pressureInnovation;				// (1x1)
  double pressureInnovation;
  Eigen::Matrix<double, 1, 1> pressureInnovationCovariance;  // (1x1)
  Eigen::Matrix<double, 1, ERROR_STATE_SIZE> pressureH;      // (1x15)
};

struct InnovationDVLStates
{
  Eigen::Matrix<double, NOMINAL_VELOCITY_STATE_SIZE, 1> DVLInnovation;
  Eigen::Matrix<double, NOMINAL_VELOCITY_STATE_SIZE, NOMINAL_VELOCITY_STATE_SIZE> DVLInnovationCovariance;
  Eigen::Matrix<double, NOMINAL_VELOCITY_STATE_SIZE, ERROR_STATE_SIZE> DVLH;
};

struct InnovationParameters
{
  size_t size;
  Eigen::MatrixXd measurementStates;
  Eigen::MatrixXd measurementCovariance;
  Eigen::MatrixXd jacobianOfErrorStates;
  explicit InnovationParameters(const size_t rows)
    : size(rows)
    , measurementStates(rows, 1)
    , measurementCovariance(rows, rows)
    , jacobianOfErrorStates(rows, ERROR_STATE_SIZE)
  {
    setZero();
  }
  void setZero()
  {
    measurementStates.setZero();
    measurementCovariance.setZero();
    jacobianOfErrorStates.setZero();
  }
};

struct parametersInESKF
{
  Eigen::Matrix<double, 3, 3> R_acc;
  Eigen::Matrix<double, 3, 3> R_accBias;
  Eigen::Matrix<double, 3, 3> R_gyro;
  Eigen::Matrix<double, 3, 3> R_gyroBias;
  Eigen::Matrix<double, 3, 3> R_dvl;
  Eigen::Matrix<double, 1, 1> R_pressureZ;
  double pgyroBias;
  double paccBias;
  Eigen::Vector3d Sr_to_ned_accelerometer;
  Eigen::Vector3d Sr_to_ned_gyro;
  Eigen::Vector3d Sr_accelerometer_aligment;
  Eigen::Vector3d Sr_gyro_aligment;
  Eigen::Vector3d Sr_to_ned_dvl;
  Eigen::Vector3d Sr_dvl_alignment;
  // Eigen::Matrix<double,3,3> S_a;
  // Eigen::Matrix<double,3,3> S_g;
  Eigen::Matrix<double, 3, 3> S_dvl;
  Eigen::Matrix<double, 3, 3> S_inc;
  Eigen::Matrix<double, NOMINAL_STATE_SIZE, 1> initial_pose;
  Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE> initial_covariance;
  bool use_ENU;
};


struct StateAndCovariance_msg
{
  double timeStamp_;
  Eigen::Matrix<double, NOMINAL_STATE_SIZE, 1> X_;
  Eigen::Matrix<double, ERROR_STATE_SIZE, ERROR_STATE_SIZE> P_;
  StateAndCovariance_msg()
  :timeStamp_{0}
  {
    X_.setZero();
    P_.setZero();
  }
  StateAndCovariance_msg(const Eigen::Matrix<double,NOMINAL_STATE_SIZE,1>& XState, const Eigen::Matrix<double,ERROR_STATE_SIZE,ERROR_STATE_SIZE>& PState,const double& timeStamp)
  :X_{XState},P_{PState},timeStamp_{timeStamp}
  {
  }

};

struct IMUmessage 
{
  bool predicted_msg_;
  double timeStamp_;
  double deltaIMU_;
  Eigen::Vector3d zAccMeasurement_;
  Eigen::Vector3d zGyroMeasurement_;
  Eigen::Matrix<double,3,3> R_acc_;
  Eigen::Matrix<double,3,3> R_gyro_;
  IMUmessage()
  :timeStamp_{0},deltaIMU_{0},predicted_msg_{false}
  {
    zAccMeasurement_.setZero();
    zGyroMeasurement_.setZero();
    R_acc_.setZero();
    R_gyro_.setZero();
  }
  IMUmessage(const double& timeStamp,const double& deltaIMU, const Eigen::Vector3d& zAcc, const Eigen::Vector3d& zGyro, const Eigen::Matrix<double,3,3>& Racc, const Eigen::Matrix<double,3,3> Rgyro)
  : timeStamp_{timeStamp},deltaIMU_{deltaIMU},zAccMeasurement_{zAcc},zGyroMeasurement_{zGyro},R_acc_{Racc},R_gyro_{Rgyro},predicted_msg_{false}
  {
  }

};

struct DVLmessage
{
  double timeStamp_;
  Eigen::Vector3d zDVl_;
  Eigen::Matrix<double,3,3> R_dvl_;
  DVLmessage()
  :timeStamp_{0}
  {
    zDVl_.setZero();
    R_dvl_.setZero();
  }
  DVLmessage(const double& timeStamp, const Eigen::Vector3d zDvl, const Eigen::Matrix<double,3,3> R_dvl)
  :timeStamp_{timeStamp},zDVl_{zDvl},R_dvl_{R_dvl}
  {
  }
};

struct PressureZmessage
{
  double timeStamp_;
  double pressureZ_msg_;
  Eigen::Matrix<double,1,1> R_pressureZ_;
  PressureZmessage()
  :timeStamp_{0}, pressureZ_msg_{0}
  {
    R_pressureZ_.setZero();
  }
  PressureZmessage(const double& timeStamp, const double& pressureZ_msg, Eigen::Matrix<double,1,1> R_pressureZ)
  :timeStamp_{timeStamp},pressureZ_msg_{pressureZ_msg},R_pressureZ_{R_pressureZ}
  {
  }
};

class ESKF
{
public:
  explicit ESKF(const parametersInESKF& parameters);

  explicit ESKF(Eigen::Matrix3d Racc, Eigen::Matrix3d RaccBias, Eigen::Matrix3d Rgyro, Eigen::Matrix3d RgyroBias, double pgyroBias, double paccBias,
                Eigen::Matrix3d Sa, Eigen::Matrix3d Sg, Eigen::Matrix3d Sdvl, Eigen::Matrix3d Sinc);
  void predictWithBuffer();
  void predict(const Eigen::Vector3d& zAccMeasurements, const Eigen::Vector3d& zGyroMeasurements, const double& Ts,
               const Eigen::Matrix3d& Racc, const Eigen::Matrix3d& Rgyro);           
  void updateDVL(const Eigen::Vector3d& zDVLvel, const Eigen::Matrix3d& RDVL);
  void updatePressureZ(const double& zPressureZpos, const Eigen::MatrixXd& RpressureZ);
  void bufferIMUMessages(const Eigen::Vector3d& zAccMeasurements, const Eigen::Vector3d& zGyroMeasurements, const double& timeStamp,const double& deltaIMU, const Eigen::Matrix3d& Racc, const Eigen::Matrix3d& Rgyro);
  void bufferDVLMessages(const Eigen::Vector3d& zDvlMeasurements,const double timeStamp,const Eigen::Matrix3d& Rdvl);
  void bufferPressureZMessages(const double& pressureZ,const double& timeStamp, Eigen::Matrix<double,1,1> R_pressureZ);
  void update();
  void UpdateOnlyWithPrediction();
  //void updateDVL(const Eigen::Vector3d& zDVLvel, const Eigen::Matrix3d& RDVL);
  //void updatePressureZ(const double& zPressureZpos, const Eigen::MatrixXd& RpressureZ);
  void updateDVLWithBuffer();
  void updatePressureZWithBuffer();

  // void setParametersInESKF(const parametersInESKF& parameters);
  inline Eigen::Quaterniond getQuaternion() const
  {
    if (use_ENU_)
    {
      return Eigen::Quaterniond{ optimizationParameters_.X(StateMemberQw), optimizationParameters_.X(StateMemberQy),
                          optimizationParameters_.X(StateMemberQx), -optimizationParameters_.X(StateMemberQz) };
    }
    else  // use NED
    {
      return Eigen::Quaterniond{ optimizationParameters_.X(StateMemberQw), optimizationParameters_.X(StateMemberQx),
                          optimizationParameters_.X(StateMemberQy), optimizationParameters_.X(StateMemberQz) };
    }
  }

  inline void setPositionAndQuaternion(Eigen::Vector3d position, Eigen::Vector4d quat)
  {

    optimizationParameters_.X.block<NOMINAL_POSITION_STATE_SIZE, 1>(NOMINAL_POSITION_STATE_OFFSET, 0) = position;
    optimizationParameters_.X.block<NOMINAL_QUATERNION_STATE_SIZE,1>(NOMINAL_QUATERNION_STATE_OFFSET,0) = quat;
  }

  
  inline Eigen::Vector3d getPosition() const
  {
    Eigen::Matrix3d R_ned_to_enu = Eigen::Matrix3d::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();

    R_ned_to_enu << 1, 0, 0, 0, -1, 0, 0, 0, 1; // TODO: Add correct transformation with EKF

    position = optimizationParameters_.X.block<NOMINAL_POSITION_STATE_SIZE, 1>(NOMINAL_POSITION_STATE_OFFSET, 0);

    if (use_ENU_)
    {
      return R_ned_to_enu * position;
    }
    else
    {
      return position;
    }
  }
  inline Eigen::Vector3d getVelocity() const
  {
    Eigen::Matrix3d R_ned_to_enu = Eigen::Matrix3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    R_ned_to_enu << 1, 0, 0, 0, -1, 0, 0, 0, 1;

    velocity = optimizationParameters_.X.block<NOMINAL_VELOCITY_STATE_SIZE, 1>(NOMINAL_VELOCITY_STATE_OFFSET, 0);

    // return R_ned_to_enu*velocity;

    if (use_ENU_)
    {
      return R_ned_to_enu * velocity;
    }
    else
    {
      return velocity;
    }
  }

  inline Eigen::Vector3d getGravity() const
  {
    return optimizationParameters_.X.block<NOMINAL_GRAVITY_SIZE, 1>(NOMINAL_GRAVITY_STATE_OFFSET, 0);
  }

  inline Eigen::VectorXd getPose() const
  {
    return optimizationParameters_.X;
  }

  inline Eigen::MatrixXd getErrorCovariance() const
  {
    return optimizationParameters_.P;
  }

  inline Eigen::Vector3d getGyroBias() const
  {
    return optimizationParameters_.X.block<NOMINAL_GYRO_BIAS_SIZE,1>(NOMINAL_GYRO_BIAS_STATE_OFFSET,0);
  }

  inline Eigen::Vector3d getAccBias() const
  {
    return optimizationParameters_.X.block<NOMINAL_ACC_BIAS_SIZE,1>(NOMINAL_ACC_BIAS_STATE_OFFSET,0);
  }

    inline double getNISPressureZ() const
  {
    return NISPressureZ_;
  }

  inline double getNISDVL() const
  {
    return NISDVL_;
  }

private:
  Eigen::VectorXd predictNominal(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                          const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts) const;

  Eigen::MatrixXd predictCovariance(const Eigen::VectorXd& xnominal, const Eigen::MatrixXd& P, const Eigen::Vector3d& accRectifiedMeasurements,
                             const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts, const Eigen::Matrix3d& Racc,
                             const Eigen::Matrix3d& Rgyro) const;

  AdandGQGD discreteErrorMatrix(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                                const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts, const Eigen::Matrix3d& Racc,
                                const Eigen::Matrix3d& Rgyro) const;
  // Lower computation time implementation
  Eigen::MatrixXd AerrDiscretizedFirstOrder(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                                     const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts) const;
  Eigen::MatrixXd AerrDiscretizedSecondOrder(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                                      const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts) const;
  Eigen::MatrixXd AerrDiscretizedThirdOrder(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                                     const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts) const;

  Eigen::Matrix3d AngularErrorMatrix(const Eigen::Vector3d& gyroRectifiedmeasurements, const double& Ts) const;

  Eigen::MatrixXd Aerr(const Eigen::VectorXd& xnominal, const Eigen::Vector3d& accRectifiedMeasurements,
                const Eigen::Vector3d& gyroRectifiedmeasurements) const;

  StatesAndErrorCovariance inject(const Eigen::VectorXd& xnominal, const Eigen::VectorXd& deltaX, const Eigen::MatrixXd& P) const;

  InnovationParameters innovationDVL(const Eigen::VectorXd& xnominal, const Eigen::MatrixXd& P, const Eigen::Vector3d& zDVLvel,
                                     const Eigen::Matrix3d& RDVL) const;
  InnovationParameters innovationDVLWithLeverArm(const Eigen::VectorXd& xnominal, const Eigen::MatrixXd& P, const Eigen::Vector3d& zDVLvel,
                                         const Eigen::Matrix3d& RDVL) const;
  static InnovationParameters innovationPressureZ(const Eigen::VectorXd& xnominal, const Eigen::MatrixXd& P,
                                                  const double& zPressureZpos, const Eigen::MatrixXd& RpressureZ);

  static Eigen::MatrixXd Gerr(const Eigen::VectorXd& xnominal);

  bool use_ENU_{ false };
  double pgyroBias_;
  double paccBias_;

  // const Eigen::Vector3d gravity_{ 0,0,GRAVITY };

  Eigen::Matrix3d Racc_;       // Acceleration measurements covariance (3x3)
  Eigen::Matrix3d RaccBias_;   // Acceleration bias driving noise covariance (3x3)
  Eigen::Matrix3d Rgyro_;      // Gyro measurements covariance (3x3)
  Eigen::Matrix3d RgyroBias_;  // Gyro bias driving noise covariance (3x3)

  // Eigen::MatrixXd D_ {blk3x3Diag(Racc_, Rgyro_, RaccBias_, RgyroBias_)};  // Diagonal block Eigen::matrix with measurement
  // covariances
  Eigen::MatrixXd D_;
  // Correction matricies
  Eigen::Matrix3d Sa_{ Eigen::Matrix3d::Zero() };    // Accelerometer
  Eigen::Matrix3d Sg_{ Eigen::Matrix3d::Zero() };    // Gyro
  Eigen::Matrix3d Sdvl_{ Eigen::Matrix3d::Zero() };  // DVL
  Eigen::Matrix3d Sinc_{ Eigen::Matrix3d::Zero() };  // Inclinometer
  double SpressureZ_{ 0 };             // Pressure

  StatesAndErrorCovariance optimizationParameters_{};

  Eigen::MatrixXd Fi();
  const Eigen::Matrix<double, ERROR_STATE_SIZE, 12> F_i_{ Fi() };
  const Eigen::MatrixXd identityMatrix_{ Eigen::MatrixXd::Identity(ERROR_STATE_SIZE, ERROR_STATE_SIZE) };
  // Execution time
  // std::vector<double> execution_time_vector_;
  bool publish_execution_time_{ false };

  // Buffer implementation
  std::vector<IMUmessage> imu_msg_buffer_;
  std::vector<DVLmessage> dvl_msg_buffer_;
  std::vector<PressureZmessage> pressureZ_msg_buffer_;
  std::vector<StateAndCovariance_msg> nominal_covariance_buffer_;
  //void emptyBuffers();
  void emptyIMUBuffer();
  void emptyDVLBuffer();
  void emptyPressureZBuffer();
  bool updated_;

  // Lever arm compensation
  Eigen::Vector3d gyro_msg_in_dvl_compensation_;

  // NIS Pressure sensor
  double NISPressureZ_;
  double NISDVL_;



};
}  // namespace eskf
