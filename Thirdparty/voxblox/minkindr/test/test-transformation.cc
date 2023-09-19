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
#include <cmath>

#include <Eigen/Dense>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>
#include <kindr/minimal/quat-transformation.h>

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

typedef kindr::minimal::QuatTransformation Transformation;

Eigen::Vector3d fromHomogeneous(const Eigen::Vector4d& v) {
  return v.head<3>() / v[3];
}


TEST(MinKindrTests, testTransform) {
  using namespace kindr::minimal;
  Eigen::Vector4d q(0.64491714, 0.26382416,  0.51605132,  0.49816637);
  RotationQuaternion q1(q[0], q[1], q[2], q[3]);
  Eigen::Vector3d t( 4.67833851,  8.52053031,  6.71796159 );

  Transformation T(q1,t);
  Transformation invT = T.inverse();
  Eigen::Vector3d v(6.26257419,  1.58356548,  6.05772983);
  Eigen::Vector4d vh(6.26257419,  1.58356548,  6.05772983, 1.0);
  Eigen::Vector4d vh2 = vh * 0.2;
  Eigen::Vector4d vh3 = -vh * 0.2;
  
  Eigen::Matrix3Xd vv(3, 2);
  vv.block<3, 1>(0, 0) = v;
  vv.block<3, 1>(0, 1) = v;
  Eigen::Matrix3Xd vempty(3, 0);

  Eigen::Vector3d Tv( 9.53512701,  15.88020996,   7.53669644);
  Eigen::Vector4d Tvh(9.53512701,  15.88020996,   7.53669644, 1.0);
  Eigen::Vector3d invTv(-6.12620997, -3.67891623,  0.04812912);
  Eigen::Vector3d Cv(4.8567885 ,  7.35967965,  0.81873485);
  Eigen::Vector3d invCv(-1.17246549,  3.4343828 ,  8.07799141);

  Eigen::Matrix4d Tmx;
  Tmx << 0.3281757 , -0.17386938,  0.92847733,  2.7527055,
         0.85444827, -0.36445416, -0.37025845,  0.7790679,
         0.40276403,  0.91484567,  0.02895739,  4.1696795,
      0.        ,  0.        ,  0.        ,  1. ;
  
  
  Eigen::Vector3d Tv1 = T.transform(v);
  Eigen::Vector4d Tvh2 = T.transform4(vh);
  Eigen::Vector4d Tvh3 = T.transform4(vh2);
  Eigen::Vector4d Tvh4 = T.transform4(vh3);
  Eigen::Vector3d Tv2 = fromHomogeneous(Tvh2);
  Eigen::Vector3d Tv3 = fromHomogeneous(Tvh3);
  Eigen::Vector3d Tv4 = fromHomogeneous(Tvh4);

  Eigen::Matrix3Xd Tvv1 = T.transformVectorized(vv);
  EXPECT_DEATH(T.transform(vempty), "^");

  for(int i = 0; i < 3; ++i) {
    EXPECT_NEAR(Tv1[i], Tv[i], 1e-4);
    EXPECT_NEAR(Tv2[i], Tv[i], 1e-4);
    EXPECT_NEAR(Tv3[i], Tv[i], 1e-4);
    EXPECT_NEAR(Tv4[i], Tv[i], 1e-4);

    EXPECT_NEAR(Tvv1(i, 0), Tv[i], 1e-4);
    EXPECT_NEAR(Tvv1(i, 1), Tv[i], 1e-4);
  }

  {
    Eigen::Vector3d invTv1 = T.inverse().transform(v);
    Eigen::Vector4d invTvh2 = T.inverse().transform4(vh);
    Eigen::Vector4d invTvh3 = T.inverse().transform4(vh2);
    Eigen::Vector4d invTvh4 = T.inverse().transform4(vh3);
    Eigen::Vector3d invTv2 = fromHomogeneous(invTvh2);
    Eigen::Vector3d invTv3 = fromHomogeneous(invTvh3);
    Eigen::Vector3d invTv4 = fromHomogeneous(invTvh4);
  
    for(int i = 0; i < 3; ++i) {
      EXPECT_NEAR(invTv1[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv2[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv3[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv4[i], invTv[i], 1e-4);
    }
  }

  {
    Eigen::Vector3d invTv1 = invT.transform(v);
    Eigen::Vector4d invTvh2 = invT.transform4(vh);
    Eigen::Vector4d invTvh3 = invT.transform4(vh2);
    Eigen::Vector4d invTvh4 = invT.transform4(vh3);
    Eigen::Vector3d invTv2 = fromHomogeneous(invTvh2);
    Eigen::Vector3d invTv3 = fromHomogeneous(invTvh3);
    Eigen::Vector3d invTv4 = fromHomogeneous(invTvh4);
  
    for(int i = 0; i < 3; ++i) {
      EXPECT_NEAR(invTv1[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv2[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv3[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv4[i], invTv[i], 1e-4);
    }
  }

  {
    Eigen::Vector3d invTv1 = T.inverseTransform(v);
    Eigen::Vector4d invTvh2 = T.inverseTransform4(vh);
    Eigen::Vector4d invTvh3 = T.inverseTransform4(vh2);
    Eigen::Vector4d invTvh4 = T.inverseTransform4(vh3);
    Eigen::Vector3d invTv2 = fromHomogeneous(invTvh2);
    Eigen::Vector3d invTv3 = fromHomogeneous(invTvh3);
    Eigen::Vector3d invTv4 = fromHomogeneous(invTvh4);
  
    for(int i = 0; i < 3; ++i) {
      EXPECT_NEAR(invTv1[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv2[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv3[i], invTv[i], 1e-4);
      EXPECT_NEAR(invTv4[i], invTv[i], 1e-4);
    }
  }
}

TEST(MinKindrTests, testCompose) {
  using namespace kindr::minimal;
  Eigen::Vector4d q(0.64491714, 0.26382416,  0.51605132,  0.49816637);
  RotationQuaternion q1(q[0], q[1], q[2], q[3]);
  Eigen::Vector3d t( 4.67833851,  8.52053031,  6.71796159 );

  Transformation T(q1,t);
  Transformation invT = T.inverse();
  Eigen::Vector3d v(6.26257419,  1.58356548,  6.05772983);
  Eigen::Vector3d invTTv(-8.16137069,  -6.14469052, -14.34176544);
  Eigen::Vector3d TTv(5.52009598,  24.34170933,  18.9197339);

  Transformation TT = T * T;
  Transformation Id1 = T * T.inverse();
  Transformation Id2 = T.inverse() * T;
  Transformation iTiT1 = T.inverse() * T.inverse();
  Transformation iTiT2 = TT.inverse();

  Eigen::Vector3d TTv1 = TT.transform(v);
  for(int i = 0; i < 3; ++i) {
    EXPECT_NEAR(TTv1[i], TTv[i],1e-4);
  }

  Eigen::Vector3d v1 = Id1.transform(v);
  Eigen::Vector3d v2 = Id2.transform(v);
  for(int i = 0; i < 3; ++i) {
    EXPECT_NEAR(v1[i], v[i],1e-4);
    EXPECT_NEAR(v2[i], v[i],1e-4);
  }

  Eigen::Vector3d iTTv1 = iTiT1.transform(v);
  Eigen::Vector3d iTTv2 = iTiT2.transform(v);
  for(int i = 0; i < 3; ++i) {
    EXPECT_NEAR(iTTv1[i], invTTv[i],1e-4);
    EXPECT_NEAR(iTTv2[i], invTTv[i],1e-4);
  }
}

TEST(MinKindrTests, testSetRandom) {
  using namespace kindr::minimal;
  Transformation T;
  T.setRandom();
  Eigen::Matrix3d R = T.getRotation().getRotationMatrix();

  // Check if orthonormal
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(R*R.transpose(), Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(MinKindrTests, testSetRandomWithNorm) {
  using namespace kindr::minimal;
  Transformation T;
  const double kTranslationNorm = 2.0;
  T.setRandom(kTranslationNorm);
  Eigen::Matrix3d R = T.getRotation().getRotationMatrix();
  Eigen::Vector3d p = T.getPosition();

  // Check if orthonormal and translation norm
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(R*R.transpose(), Eigen::Matrix3d::Identity(), 1e-6));
  EXPECT_NEAR(p.norm(), kTranslationNorm, 1e-8);
}

TEST(MinKindrTests, testSetRandomWithAngleAndNorm) {
  using namespace kindr::minimal;
  Transformation T;
  const double kTranslationNorm = 2.0;
  const double KRotationAngleRad = 10.0 / 180.0 * M_PI;

  T.setRandom(kTranslationNorm, KRotationAngleRad);
  Eigen::Matrix3d R = T.getRotation().getRotationMatrix();
  Eigen::Vector3d p = T.getPosition();

  // Check if orthonormal, translation norm and rotation angle
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(R*R.transpose(), Eigen::Matrix3d::Identity(), 1e-6));
  EXPECT_NEAR(p.norm(), kTranslationNorm, 1e-8);
  EXPECT_NEAR(AngleAxis(R).angle(), KRotationAngleRad, 1e-8);
}

TEST(MinKindrTests, testExpLog) {
  using namespace kindr::minimal;
  for(int i = 0; i < 10; ++i) {
    Transformation T1;
    T1.setRandom();
    Transformation::Vector6 v = T1.log();
    Transformation T2 = Transformation::exp(v);
    Eigen::Matrix4d TT1 = T1.getTransformationMatrix();
    Eigen::Matrix4d TT2 = T2.getTransformationMatrix();
    for(int r = 0; r < 4; ++r) {
      for(int c = 0; c < 4; ++c) {
        EXPECT_NEAR(TT1(r,c), TT2(r,c), 1e-6) << "Failed at (" << r << "," << c << ")";
      }
    }
  }
}
