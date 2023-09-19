// Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <gtest/gtest.h>
#include <kindr/minimal/rotation-quaternion.h>
#include <kindr/minimal/angle-axis.h>
#include <cmath>
#include <eigen-checks/gtest.h>

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

TEST(MinKindrTests,testQuatAxisAngle) {
  using namespace kindr::minimal;
  Eigen::Vector4d v(0.64491714, 0.26382416,  0.51605132,  0.49816637);
  Eigen::Matrix3d C;

  // This is data generated from a trusted python implementation.
  C << -0.02895739, -0.37025845,  0.92847733,
        0.91484567,  0.36445417,  0.17386938,
       -0.40276404,  0.85444827,  0.3281757;

  Eigen::Vector4d aax(1.7397629325686206, 0.34520549,  0.67523668,  0.65183479);

  RotationQuaternion q1(v[0], v[1], v[2], v[3]);
  RotationQuaternion q2(C);
  RotationQuaternion q3(v[0],v.tail<3>());
  RotationQuaternion q4(v[0], v[1], v[2], v[3]);
  RotationQuaternion q5(-v[0], -v[1], -v[2], -v[3]);


  AngleAxis a1(aax[0],aax.tail<3>()); 
  AngleAxis a2(aax[0] + 2*M_PI,aax.tail<3>()); 
  AngleAxis a3(aax[0] + 4*M_PI,aax.tail<3>()); 
  AngleAxis a4(q1);
  Eigen::Vector3d rotation_vector = aax[0]*aax.tail<3>();
  AngleAxis a5(rotation_vector);

  RotationQuaternion q6(a1);

  EXPECT_NEAR(q1.getDisparityAngle(q2), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(q3), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(q4), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(q5), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(q6), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(a1), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(a2), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(a2), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(a3), 0.0, 1e-3);
  EXPECT_NEAR(q1.getDisparityAngle(a4), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q2), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q3), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q4), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q5), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q6), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(q1), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(a2), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(a2), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(a3), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(a4), 0.0, 1e-3);
  EXPECT_NEAR(a1.getDisparityAngle(a5), 0.0, 1e-3);

  EXPECT_NEAR(q1.inverse().getDisparityAngle(q2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(q3.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(q4.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(q5.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(q6.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(a1.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(a2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(a2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(a3.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(q1.inverse().getDisparityAngle(a4.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q3.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q4.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q5.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q6.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(q1.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(a2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(a2.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(a3.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(a4.inverse()), 0.0, 1e-3);
  EXPECT_NEAR(a1.inverse().getDisparityAngle(a5.inverse()), 0.0, 1e-3);


}

TEST(MinKindrTests,testComposition) {
  using namespace kindr::minimal;
  Eigen::Vector4d v(0.64491714, 0.26382416,  0.51605132,  0.49816637);
  Eigen::Matrix3d C, Csquared;

  // This is data generated from a trusted python implementation.
  C << -0.02895739, -0.37025845,  0.92847733,
        0.91484567,  0.36445417,  0.17386938,
       -0.40276404,  0.85444827,  0.3281757;

  Csquared << -0.71184809,  0.66911533,  0.21344081,
      0.23689944, -0.05734011,  0.96984059,
      0.66117392,  0.74094318, -0.1176956;

  Eigen::Vector4d aax(1.7397629325686206, 0.34520549,  0.67523668,  0.65183479);
  
  
  RotationQuaternion q1(v[0], v[1], v[2], v[3]);
  AngleAxis a1(aax[0],aax.tail<3>()); 


  RotationQuaternion qsquared(Csquared); 
  
  EXPECT_NEAR((q1*q1).getDisparityAngle(qsquared), 0.0, 1e-3);
  EXPECT_NEAR((q1*a1).getDisparityAngle(qsquared), 0.0, 1e-3);
  EXPECT_NEAR((a1*q1).getDisparityAngle(qsquared), 0.0, 1e-3);
  EXPECT_NEAR((a1*a1).getDisparityAngle(qsquared), 0.0, 1e-3);
  EXPECT_NEAR((a1.inverse()*a1.inverse()).getDisparityAngle(qsquared.inverse()), 0.0, 1e-3);
  EXPECT_NEAR((q1.inverse()*a1.inverse()).getDisparityAngle(qsquared.inverse()), 0.0, 1e-3);
  EXPECT_NEAR((a1.inverse()*q1.inverse()).getDisparityAngle(qsquared.inverse()), 0.0, 1e-3);
  EXPECT_NEAR((q1.inverse()*q1.inverse()).getDisparityAngle(qsquared.inverse()), 0.0, 1e-3);

}

TEST(MinKindrTests, testQuaternionInitialization) {
 Eigen::Vector4d q_coeffs;
 q_coeffs << 1, 0, 0, 0;
 kindr::minimal::RotationQuaternionTemplate<double> q_from_coeffs(
     q_coeffs[0], q_coeffs[1], q_coeffs[2], q_coeffs[3]);
 Eigen::Quaterniond q;
 q.setIdentity();

 for(int i = 0; i < 4; ++i) {
    EXPECT_NEAR(q_from_coeffs.toImplementation().coeffs()[i],
                q.coeffs()[i], 1e-10);
 }
}

TEST(MinKindrTests, testRotate) {
  using namespace kindr::minimal;
  Eigen::Vector4d q(0.64491714, 0.26382416,  0.51605132,  0.49816637);
  Eigen::Matrix3d C, Csquared;

  // This is data generated from a trusted python implementation.
  C << -0.02895739, -0.37025845,  0.92847733,
        0.91484567,  0.36445417,  0.17386938,
       -0.40276404,  0.85444827,  0.3281757;

  Eigen::Vector4d aax(1.7397629325686206, 0.34520549,  0.67523668,  0.65183479);
  
  
  RotationQuaternion q1(q[0], q[1], q[2], q[3]);
  AngleAxis a1(aax[0],aax.tail<3>()); 

  Eigen::Vector3d v(4.67833851,  8.52053031,  6.71796159);
  Eigen::Vector3d Cv(2.94720425,  8.55334831,  7.60075758);
  Eigen::Vector3d Ctv(4.95374446,  7.11329907,  8.02986227);
  Eigen::Vector3d Cv1 = q1.rotate(v);
  Eigen::Vector3d Cv2 = a1.rotate(v);
  Eigen::Vector3d Ctv1 = q1.inverseRotate(v);
  Eigen::Vector3d Ctv2 = a1.inverseRotate(v);
  Eigen::Vector3d Ctv3 = q1.inverse().rotate(v);
  Eigen::Vector3d Ctv4 = a1.inverse().rotate(v);


  for(int i = 0; i < 3; ++i) {
     EXPECT_NEAR(Cv[i], Cv1[i], 1e-4);
     EXPECT_NEAR(Cv[i], Cv2[i], 1e-4);
     EXPECT_NEAR(Ctv[i], Ctv1[i], 1e-4);
     EXPECT_NEAR(Ctv[i], Ctv2[i], 1e-4);
     EXPECT_NEAR(Ctv[i], Ctv3[i], 1e-4);
     EXPECT_NEAR(Ctv[i], Ctv4[i], 1e-4);
  }
}

TEST(MinKindrTests, testRotationExpLog) {
  using namespace kindr::minimal;
  for(int i = 0; i < 10; ++i) {
    RotationQuaternion C1;
    C1.setRandom();
    RotationQuaternion::Vector3 v = C1.log();
    RotationQuaternion C2 = RotationQuaternion::exp(v);
    Eigen::Matrix3d CC1 = C1.getRotationMatrix();
    Eigen::Matrix3d CC2 = C2.getRotationMatrix();
    for(int r = 0; r < 3; ++r) {
      for(int c = 0; c < 3; ++c) {
        EXPECT_NEAR(CC1(r,c), CC2(r,c), 1e-6) << "Failed at (" << r << "," << c << ")";
      }
    }
  }

  RotationQuaternion::Vector3 axis;
  axis << 0.0,0.0,1.0;
  axis /= axis.norm();
  for (double angle = -M_PI; angle <= M_PI; angle+=M_PI/100) {
    RotationQuaternion::Vector3 axisAngle = axis*angle;
    RotationQuaternion C2 = RotationQuaternion::exp(axisAngle);
    RotationQuaternion::Vector3 v = C2.log();
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(v,axisAngle,1e-6));
  }

  Eigen::Matrix<double,4,1> q;
  for(int i=0; i < 1000; ++i) {
    RotationQuaternion C1;
    C1.setRandom();
    RotationQuaternion::Vector3 v1 = C1.log();
    C1.setParts(-C1.w(), -C1.imaginary());
    RotationQuaternion::Vector3 v2 = C1.log();
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(v1,v2,1e-6));
  }
}
