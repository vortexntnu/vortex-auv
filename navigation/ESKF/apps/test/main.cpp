#include "ESKF.h"
#include <chrono>

using namespace Eigen;

int main(int argc, char *argv[])
{
  /*
  ros::init(argc,argv,"eskf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ESKF_Node eskf_node(nh,pnh);
  ros::spin();
  return 0;
  */

  // Matrix3d test = Matrix3d::Zero();

  // double* arraytesting= test.data();

  // arraytesting[0] = 2;

  // std::cout<<arraytesting[0]<<std::endl;

  MatrixXd INITIAL_P(15, 15);
  INITIAL_P.setIdentity();

  // InjectionStates injectionDVLTesting;
  eskf::InnovationParameters dvlstatetesting{ 3 };
  Vector3d ZdvlValues = Vector3d::Zero();
  Matrix3d RDVL = Matrix3d::Zero();
  // InjectionStates injectionPressureTesting;
  // StatePredictions statetesting;
  eskf::InnovationParameters pressureTesting{ 1 };
  VectorXd deltaX(15);
  // InjectionStates injectiontesting;
  MatrixXd P(15, 15);
  eskf::AdandGQGD testing;
  double Ts{ 0.0080 };
  Vector3d accRectifiedMeasurements = Vector3d::Zero();
  Vector3d gyroRectifiedMeasurements = Vector3d::Zero();
  VectorXd xnominal(16);
  Matrix3d Racc = Matrix3d::Zero();
  Matrix3d RaccBias = Matrix3d::Zero();
  Matrix3d Rgyro = Matrix3d::Zero();
  Matrix3d RgyroBias = Matrix3d::Zero();
  double pgyroBias{ 0.0001 };
  double paccBias{ 0.0001 };
  Matrix3d Sa = Matrix3d::Zero();
  Matrix3d Sg = Matrix3d::Zero();
  Matrix3d Sdvl = Matrix3d::Zero();
  Matrix3d Sinc = Matrix3d::Zero();
  double SpressureZ;
  double pressureValue{ 0.5 };
  MatrixXd RpressureZ(1, 1);

  P.setIdentity();

  xnominal << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  deltaX << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

  ZdvlValues << 1, 2, 3;
  RDVL.setIdentity();

  RDVL = RDVL * 2.0;

  Racc << 3, 0, 0, 0, 3, 0, 0, 0, 3;

  RaccBias << 6e-5, 0, 0, 0, 6e-5, 0, 0, 0, 6e-5;

  Rgyro << 0.0120, 0, 0, 0, 0.0120, 0, 0, 0, 0.0120;

  RgyroBias << 3e-7, 0, 0, 0, 3e-7, 0, 0, 0, 3e-7;

  RpressureZ << 0.5;

  Sa.setIdentity();
  Sg.setIdentity();

  accRectifiedMeasurements << 4, 5, 2;
  gyroRectifiedMeasurements << 4, 5, 2;

  eskf::ESKF eskf{ Racc, RaccBias, Rgyro, RgyroBias, pgyroBias, paccBias, Sa, Sg, Sdvl, Sinc };

  Racc = Racc * Ts;

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 10000; ++i)
  {
    eskf.predict(accRectifiedMeasurements, gyroRectifiedMeasurements, Ts, Racc, Rgyro);
  }
  auto end = std::chrono::steady_clock::now();
  auto diff = end - start;
  std::cout << "predictnominal: " << std::chrono::duration<double, std::milli>(diff).count() << " ms" << std::endl;

  // Execution time
  // auto end = std::chrono::steady_clock::now();

  // auto diff = end - start;

  // auto diff_in_ms = std::chrono::duration <double, std::milli> (diff).count();

  // std::cout<<diff_in_ms<<std::endl;
  /*
  if(execution_time_vector_.size() == 1000 && publish_execution_time_ == true)
  {
    std::cout<<"Max value: "<<maxOfVector(execution_time_vector_)<<std::endl;
    std::cout<<"Mean: "<<meanOfVector(execution_time_vector_)<<std::endl;
    std::cout<<"STD: "<<stanardDeviationOfVector(execution_time_vector_)<<std::endl;
    publish_execution_time_ = false;
  }
  else
  {
    execution_time_vector_.push_back(diff_in_ms);
  }
  */
  // pressureTesting = eskf.innovationPressureZ(xnominal, P, pressureValue, RpressureZ);
  // injectionPressureTesting = eskf.updatePressureZ(xnominal, P, pressureValue, RpressureZ);
  // dvlstatetesting = eskf.innovationDVL(xnominal, P, ZdvlValues, RDVL);
  // injectionDVLTesting = eskf.updateDVL(xnominal, P, ZdvlValues, RDVL);

  // std::cout << injectionDVLTesting.pInject << std::endl;
  // std::cout << injectionDVLTesting.xInject << std::endl;

  // std::cout << dvlstatetesting.DVLH << std::endl;
  // std::cout << dvlstatetesting.DVLInnovation << std::endl;
  // std::cout << dvlstatetesting.DVLInnovationCovariance << std::endl;

  // std::cout << pressureTesting.pressureH << std::endl;
  // std::cout << pressureTesting.pressureInnovation << std::endl;
  // std::cout << pressureTesting.pressureInnovationCovariance << std::endl;

  // std::cout << injectionPressureTesting.xInject << std::endl;
  // std::cout << injectionPressureTesting.pInject << std::endl;

  // statetesting = eskf.predict(xnominal, P, accRectifiedMeasurements, gyroRectifiedMeasurements, Ts);
  // injectiontesting = eskf.inject(xnominal, deltaX, P);
  // testing = eskf.discreteErrorMatrix(xnominal, accRectifiedMeasurements, gyroRectifiedMeasurements, Ts);

  // std::cout << eskf.predictNominal(xnominal,accRectifiedMeasurements,gyroRectifiedMeasurements,Ts)<<std::endl;
  // std::cout << eskf.Aerr(xnominal,accRectifiedMeasurements,gyroRectifiedMeasurements)<<std::endl;
  // std::cout << eskf.Gerr(xnominal)<<std::endl;
  // std::cout << testing.Ad <<std::endl;
  // std::cout << testing.GQGD << std::endl;
  // eskf.discreteErrorMatrix(xnominal, accRectifiedMeasurements, gyroRectifiedMeasurements, Ts);

  // std::cout << eskf.predictCovariance(xnominal,P,accRectifiedMeasurements,gyroRectifiedMeasurements,Ts) <<std::endl;
  // std::cout << injectiontesting.xInject << std::endl;
  // std::cout << statetesting.xNominalPrediction << std::endl;

  return 0;
}