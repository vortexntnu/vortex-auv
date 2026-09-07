#include <gtest/gtest.h>
#include <cmath>
#include <geometry_msgs/msg/detail/wrench_stamped__struct.hpp>
#include "velocity_controller/utilities.hpp"

// ---------- quaternion_to_euler_angle ----------

TEST(QuaternionToEulerTest, IdentityQuaternionGivesZeroAngles) {
    angle a = quaternion_to_euler_angle(1.0, 0.0, 0.0, 0.0);
    EXPECT_NEAR(a.phit, 0.0, 1e-9);
    EXPECT_NEAR(a.thetat, 0.0, 1e-9);
    EXPECT_NEAR(a.psit, 0.0, 1e-9);
}

TEST(QuaternionToEulerTest, NinetyDegreeYawQuaternion) {
    // Kvaternion for 90 grader (pi/2) rotasjon om z-aksen
    double w = std::cos(M_PI / 4.0);
    double z = std::sin(M_PI / 4.0);
    angle a = quaternion_to_euler_angle(w, 0.0, 0.0, z);
    EXPECT_NEAR(a.phit, 0.0, 1e-9);
    EXPECT_NEAR(a.thetat, 0.0, 1e-9);
    EXPECT_NEAR(a.psit, M_PI / 2.0, 1e-9);
}

TEST(QuaternionToEulerTest, GimbalLockClampingDoesNotProduceNaN) {
    // t2 klippes til [-1, 1] - test at en verdi som ville gitt |t2| > 1
    // pga flyttallsavrunding ikke gir NaN fra asin()
    double w = std::sqrt(0.5);
    double y = std::sqrt(0.5);
    angle a = quaternion_to_euler_angle(w, 0.0, y, 0.0);
    EXPECT_FALSE(std::isnan(a.thetat));
}

// ---------- angle_NED_to_body ----------

TEST(AngleNEDToBodyTest, IdenticalAttitudesGiveZeroError) {
    angle result = angle_NED_to_body(0.3, 0.2, 0.5, 0.3, 0.2, 0.5);
    EXPECT_NEAR(result.phit, 0.0, 1e-9);
    EXPECT_NEAR(result.thetat, 0.0, 1e-9);
    EXPECT_NEAR(result.psit, 0.0, 1e-9);
}

TEST(AngleNEDToBodyTest, YawOnlyDifferenceGivesCurrentMinusDesired) {
    // roll=pitch=0 begge steder, kun yaw differerer.
    // Håndregnet: yaw_err = actual_yaw - desired_yaw (IKKE desired - actual)
    double desired_yaw = 0.4;
    double actual_yaw = 1.0;
    angle result = angle_NED_to_body(0.0, 0.0, desired_yaw, 0.0, 0.0, actual_yaw);
    EXPECT_NEAR(result.phit, 0.0, 1e-9);
    EXPECT_NEAR(result.thetat, 0.0, 1e-9);
    EXPECT_NEAR(result.psit, actual_yaw - desired_yaw, 1e-9);
}

TEST(AngleNEDToBodyTest, PitchOnlyDifferenceGivesCurrentMinusDesired) {
    // roll=yaw=0 begge steder, kun pitch differerer.
    // Håndregnet: pitch_err = actual_pitch - desired_pitch
    double desired_pitch = 0.25;
    double actual_pitch = -0.35;
    angle result = angle_NED_to_body(0.0, desired_pitch, 0.0, 0.0, actual_pitch, 0.0);
    EXPECT_NEAR(result.phit, 0.0, 1e-9);
    EXPECT_NEAR(result.thetat, actual_pitch - desired_pitch, 1e-9);
    EXPECT_NEAR(result.psit, 0.0, 1e-9);
}

// NB: kombinerte flerakse-avvik (f.eks roll+pitch+yaw samtidig) involverer
// kobling mellom aksene i R_error-ekstraksjonen, og er for feilutsatt å
// håndregne pålitelig her. Anbefaler: kjør en slik test én gang, inspiser
// utskrevet resultat manuelt (evt. sammenlign mot en referanseimplementasjon
// som scipy.spatial.transform.Rotation), og lås verdien først da.

// ---------- wrench_to_vector / vector_to_wrench roundtrip ----------

TEST(WrenchVectorConversionTest, RoundTripPreservesValues) {
    geometry_msgs::msg::WrenchStamped w;
    w.wrench.force.x = 1.5; w.wrench.force.y = -2.5; w.wrench.force.z = 3.5;
    w.wrench.torque.x = 0.1; w.wrench.torque.y = -0.2; w.wrench.torque.z = 0.3;

    Eigen::Vector<double, 6> vec = wrench_to_vector(w);
    geometry_msgs::msg::WrenchStamped w2 = vector_to_wrench(vec);

    EXPECT_DOUBLE_EQ(w2.wrench.force.x, w.wrench.force.x);
    EXPECT_DOUBLE_EQ(w2.wrench.force.y, w.wrench.force.y);
    EXPECT_DOUBLE_EQ(w2.wrench.force.z, w.wrench.force.z);
    EXPECT_DOUBLE_EQ(w2.wrench.torque.x, w.wrench.torque.x);
    EXPECT_DOUBLE_EQ(w2.wrench.torque.y, w.wrench.torque.y);
    EXPECT_DOUBLE_EQ(w2.wrench.torque.z, w.wrench.torque.z);
}

TEST(WrenchVectorConversionTest, VectorOrderMatchesExpectedLayout) {
    Eigen::Vector<double, 6> vec;
    vec << 1, 2, 3, 4, 5, 6;
    geometry_msgs::msg::WrenchStamped w = vector_to_wrench(vec);
    EXPECT_DOUBLE_EQ(w.wrench.force.x, 1);
    EXPECT_DOUBLE_EQ(w.wrench.force.y, 2);
    EXPECT_DOUBLE_EQ(w.wrench.force.z, 3);
    EXPECT_DOUBLE_EQ(w.wrench.torque.x, 4);
    EXPECT_DOUBLE_EQ(w.wrench.torque.y, 5);
    EXPECT_DOUBLE_EQ(w.wrench.torque.z, 6);
}

// ---------- coriolis ----------

TEST(CoriolisTest, ZeroStateGivesZeroMatrix) {
    State s{};  // alt 0
    auto C = coriolis(s, /*mass=*/30.0, /*Ixx=*/2.0, /*Iyy=*/3.0, /*Izz=*/3.0);
    EXPECT_TRUE(C.isZero(1e-12));
}

TEST(CoriolisTest, MatrixIsSkewSymmetric) {
    // Fysisk egenskap: rigid-body Coriolis-matrisen på denne formen skal
    // være skjev-symmetrisk (C = -C^T) uansett input. Dette er en sterkere
    // og mer robust test enn å håndregne hver av de 12 fylte cellene enkeltvis -
    // den fanger opp fortegnsfeil introdusert i FREMTIDIGE endringer også.
    State s{};
    s.surge = 1.2; s.sway = -0.7; s.heave = 0.3;
    s.roll_rate = 0.4; s.pitch_rate = -0.6; s.yaw_rate = 0.9;

    auto C = coriolis(s, /*mass=*/30.0, /*Ixx=*/2.0, /*Iyy=*/3.0, /*Izz=*/3.5);
    Eigen::Matrix<double, 6, 6> sum = C + C.transpose();
    EXPECT_TRUE(sum.isZero(1e-9))
        << "Coriolis-matrisen skal være skjev-symmetrisk (C + C^T = 0). "
           "Sum-matrise:\n" << sum;
}

TEST(CoriolisTest, SpecificEntriesMatchExpectedFormula) {
    // Direkte verdisjekk på et par celler som regresjonsvern for selve formelen
    State s{};
    s.surge = 2.0; s.sway = 1.0; s.heave = 0.5;
    s.roll_rate = 0.1; s.pitch_rate = 0.2; s.yaw_rate = 0.3;
    double mass = 30.0, Ixx = 2.0, Iyy = 3.0, Izz = 3.5;

    auto C = coriolis(s, mass, Ixx, Iyy, Izz);

    EXPECT_DOUBLE_EQ(C(0, 4), mass * s.heave);
    EXPECT_DOUBLE_EQ(C(0, 5), -mass * s.sway);
    EXPECT_DOUBLE_EQ(C(3, 4), Izz * s.yaw_rate);
    EXPECT_DOUBLE_EQ(C(4, 5), Ixx * s.roll_rate);
}