#include <gtest/gtest.h>
#include <eigen-checks/gtest.h>

#include "kindr/minimal/quat-sim-transform.h"

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

namespace kindr {
namespace minimal {

TEST(QuatSimTransformTest, Transform) {
  Eigen::Matrix3Xd input(3, 1);
  input << 1., 1., 0.;
  Eigen::Matrix4d transformation_matrix;
  transformation_matrix << 1, 0,  0, 0,
                           0, 0, -1, 1,
                           0, 1,  0, 0,
                           0, 0,  0, 1;
  const QuatSimTransform sim3(QuatTransformation(
      transformation_matrix), 2.0);
  Eigen::Matrix3Xd expected_output(3, 1);
  expected_output << 2., 1., 2.;

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(sim3 * input, expected_output));
}

TEST(QuatSimTransformTest, Inverse) {
  const RotationQuaternion q(Eigen::Quaterniond(
      0.64491714, 0.26382416, 0.51605132, 0.49816637));
  const Eigen::Vector3d t(4.67833851, 8.52053031, 6.71796159 );
  const QuatSimTransform sim3(QuatTransformation(q, t), M_PI);

  // 3Xd intended - it's the input type to operator*.
  const Eigen::Matrix3Xd input = Eigen::Matrix3d::Random();

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      sim3 * (sim3.inverse() * input), input, 1e-6));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      sim3.inverse() * (sim3 * input), input, 1e-6));
}

}  // namespace minimal
}  // namespace kindr
