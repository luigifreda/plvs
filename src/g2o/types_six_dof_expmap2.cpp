/*
 * This file is part of PLVS
 * Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

// EdgeSE3ProjectLineOnlyPose (unary edge to optimize only camera pose )
// EdgeSE3ProjectStereoLineOnlyPose (unary edge to optimize only camera pose )
// EdgeSE3ProjectLine (binary edge to optimize camera pose and line representation)
// EdgeSE3ProjectStereoLine (binary edge to optimize camera pose and line representation)

#include "g2o/types_six_dof_expmap2.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

namespace g2o {

using namespace std;
using namespace Eigen;


static Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

static Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 
/// < < < < < <  < < < < <  NEW RGBD POINTS < < < < <  < < < < <  < < < < < 
/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 

Vector3d EdgeRgbdSE3ProjectXYZOnlyPose::camProject(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = trans_xyz[2];
  return res;
}


bool EdgeRgbdSE3ProjectXYZOnlyPose::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeRgbdSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeRgbdSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  const Vector3d xyz_trans = vi->estimate().map(Xw);

  const double x = xyz_trans[0];
  const double y = xyz_trans[1];
  const double invz = 1.0/xyz_trans[2];
  const double invz_2 = invz*invz;

//  Jprojxyz =
//[     (fx*xc*yc)/zc^2, - fx - (fx*xc^2)/zc^2,  (fx*yc)/zc, -fx/zc,      0, (fx*xc)/zc^2]
//[ fy + (fy*yc^2)/zc^2,      -(fy*xc*yc)/zc^2, -(fy*xc)/zc,      0, -fy/zc, (fy*yc)/zc^2]
//[                 -yc,                    xc,           0,      0,      0,           -1]
          
  _jacobianOplusXi(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXi(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXi(0,2) = y*invz *fx;
  _jacobianOplusXi(0,3) = -invz *fx;
  _jacobianOplusXi(0,4) = 0.;
  _jacobianOplusXi(0,5) = x*invz_2 *fx;

  _jacobianOplusXi(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXi(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXi(1,2) = -x*invz *fy;
  _jacobianOplusXi(1,3) = 0;
  _jacobianOplusXi(1,4) = -invz *fy;
  _jacobianOplusXi(1,5) = y*invz_2 *fy;

  _jacobianOplusXi(2,0) = -y;
  _jacobianOplusXi(2,1) =  x;
  _jacobianOplusXi(2,2) =  0.;
  _jacobianOplusXi(2,3) =  0.;
  _jacobianOplusXi(2,4) =  0.;
  _jacobianOplusXi(2,5) = -1.;
}

// -- -- -- --

Vector3d EdgeRgbdSE3ProjectXYZ::camProject(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = trans_xyz[2];
  return res;
}

EdgeRgbdSE3ProjectXYZ::EdgeRgbdSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeRgbdSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<=3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeRgbdSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<=3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<=2; i++)
    for (int j=i; j<=2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeRgbdSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  const SE3Quat& T = vj->estimate();
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  const Vector3d& xyz = vi->estimate();
  const Vector3d xyz_trans = T.map(xyz);

  const Matrix3d R =  T.rotation().toRotationMatrix();

  const double x = xyz_trans[0];
  const double y = xyz_trans[1];

  const double invz = 1.0/xyz_trans[2];
  const double invz_2 = invz*invz;  

  _jacobianOplusXi(0,0) = -fx*R(0,0)*invz+fx*x*R(2,0)*invz_2;
  _jacobianOplusXi(0,1) = -fx*R(0,1)*invz+fx*x*R(2,1)*invz_2;
  _jacobianOplusXi(0,2) = -fx*R(0,2)*invz+fx*x*R(2,2)*invz_2;

  _jacobianOplusXi(1,0) = -fy*R(1,0)*invz+fy*y*R(2,0)*invz_2;
  _jacobianOplusXi(1,1) = -fy*R(1,1)*invz+fy*y*R(2,1)*invz_2;
  _jacobianOplusXi(1,2) = -fy*R(1,2)*invz+fy*y*R(2,2)*invz_2;

  _jacobianOplusXi(2,0) = -R(2,0);
  _jacobianOplusXi(2,1) = -R(2,1);
  _jacobianOplusXi(2,2) = -R(2,2);

//  Jprojxyz =
//[     (fx*xc*yc)/zc^2, - fx - (fx*xc^2)/zc^2,  (fx*yc)/zc, -fx/zc,      0, (fx*xc)/zc^2]
//[ fy + (fy*yc^2)/zc^2,      -(fy*xc*yc)/zc^2, -(fy*xc)/zc,      0, -fy/zc, (fy*yc)/zc^2]
//[                 -yc,                    xc,           0,      0,      0,           -1]
  _jacobianOplusXj(0,0) =  x*y*invz_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x*invz_2)) *fx;
  _jacobianOplusXj(0,2) = y*invz *fx;
  _jacobianOplusXj(0,3) = -invz *fx;
  _jacobianOplusXj(0,4) = 0.;
  _jacobianOplusXj(0,5) = x*invz_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y*invz_2) *fy;
  _jacobianOplusXj(1,1) = -x*y*invz_2 *fy;
  _jacobianOplusXj(1,2) = -x*invz *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -invz *fy;
  _jacobianOplusXj(1,5) = y*invz_2 *fy;

  _jacobianOplusXj(2,0) = -y;
  _jacobianOplusXj(2,1) =  x;
  _jacobianOplusXj(2,2) =  0.;
  _jacobianOplusXj(2,3) =  0.;
  _jacobianOplusXj(2,4) =  0.;
  _jacobianOplusXj(2,5) = -1.;  
}

/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 
/// < < < < < <  < < < < <   LINES     < < < < <  < < < < <  < < < < < 
/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 


bool EdgeSE3ProjectLineOnlyPose::read(std::istream& is){
  for (int i=0; i<3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectLineOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

#if USE_ANALYTIC_JACS_ONLY_POSE_MONO 
void EdgeSE3ProjectLineOnlyPose::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  const Vector3d xyz_trans_s = vi->estimate().map(XSw);
  const Vector3d xyz_trans_e = vi->estimate().map(XEw);

  const double& l1 = _measurement[0];
  const double& l2 = _measurement[1];
  //const double& l3 = _measurement[2];
  
  const double& x1 = xyz_trans_s[0];
  const double& y1 = xyz_trans_s[1];
  const double invz1 = 1.0/xyz_trans_s[2];
  const double invz1_2 = invz1*invz1;
  
  const double& x2 = xyz_trans_e[0];
  const double& y2 = xyz_trans_e[1]; 
  const double invz2 = 1.0/xyz_trans_e[2];
  const double invz2_2 = invz2*invz2;
  
  const double fxl1 = fx*l1;
  const double fyl2 = fy*l2;

/// < N.B.: we take the following Jacobian Ji as is without inverting the sign (we are dealing with a constraint h=0)  
//[ - fy*l2 - y1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//    fx*l1 + x1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//   (fy*l2*x1)/z1 - (fx*l1*y1)/z1, 
//   (fx*l1)/z1, 
//   (fy*l2)/z1, 
// - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
//        
//[ - fy*l2 - y2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//    fx*l1 + x2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//   (fy*l2*x2)/z2 - (fx*l1*y2)/z2, 
//   (fx*l1)/z2, 
//   (fy*l2)/z2, 
// - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
 
  const double term1 = ( (fxl1*x1)+ (fyl2*y1) )*invz1_2;
  
  _jacobianOplusXi(0,0) = -fyl2 - y1*(term1);
  _jacobianOplusXi(0,1) =  fxl1 + x1*(term1);
  _jacobianOplusXi(0,2) = (fyl2*x1)*invz1 - (fxl1*y1)*invz1;
  _jacobianOplusXi(0,3) = (fxl1)*invz1;
  _jacobianOplusXi(0,4) = (fyl2)*invz1; 
  _jacobianOplusXi(0,5) = -(fxl1*x1)*invz1_2 - (fyl2*y1)*invz1_2;
  
  const double term2 = ( (fxl1*x2) + (fyl2*y2) )*invz2_2;
  
  _jacobianOplusXi(1,0) = -fyl2 - y2*(term2);
  _jacobianOplusXi(1,1) =  fxl1 + x2*(term2); 
  _jacobianOplusXi(1,2) = (fyl2*x2)*invz2 - (fxl1*y2)*invz2;
  _jacobianOplusXi(1,3) = (fxl1)*invz2; 
  _jacobianOplusXi(1,4) = (fyl2)*invz2;
  _jacobianOplusXi(1,5) = -(fxl1*x2)*invz2_2 - (fyl2*y2)*invz2_2;
 
}
#endif


Vector3d EdgeSE3ProjectLineOnlyPose::camProject(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector3d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  res[2] = 1; 
  return res;
}

Vector2d EdgeSE3ProjectLineOnlyPose::camProject2d(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}


// -- -- -- --


bool EdgeSE3ProjectStereoLineOnlyPose::read(std::istream& is){
  for (int i=0; i<5; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<5; i++)
    for (int j=i; j<5; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectStereoLineOnlyPose::write(std::ostream& os) const {

  for (int i=0; i<5; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<5; i++)
    for (int j=i; j<5; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

  
Vector2d EdgeSE3ProjectStereoLineOnlyPose::camProject(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector2d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  return res;
}

Vector3d EdgeSE3ProjectStereoLineOnlyPose::camBackProject(const Vector2d & p_uv, const double depth ) const{
  Vector3d res;
  res[0] = (p_uv[0]-cx)*depth/fx;
  res[1] = (p_uv[1]-cy)*depth/fy; 
  res[2] = depth;
  return res;
}

#if USE_ANALYTIC_JACS_ONLY_POSE_STEREO
void EdgeSE3ProjectStereoLineOnlyPose::linearizeOplus() {
  const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]);
  const SE3Quat& T = vi->estimate();
  //const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
  const Vector3d xyz_trans_s = T.map(XSw);
  const Vector3d xyz_trans_e = T.map(XEw);
  
  const double& l1 = _measurement[0];
  const double& l2 = _measurement[1];
  //const double& l3 = _measurement[2];
  
  const double& x1 = xyz_trans_s[0];
  const double& y1 = xyz_trans_s[1];
  const double& z1 = xyz_trans_s[2];
  const double invz1 = 1.0/xyz_trans_s[2];
  const double invz1_2 = invz1*invz1;
  
  const double& x2 = xyz_trans_e[0];
  const double& y2 = xyz_trans_e[1];
  const double& z2 = xyz_trans_e[2];  
  const double invz2 = 1.0/xyz_trans_e[2];
  const double invz2_2 = invz2*invz2;
  
  const double fxl1 = fx*l1;
  const double fyl2 = fy*l2;
  
  const Vector3d& Pc = xyz_trans_s;
  const Vector3d& Qc = xyz_trans_e;
  
  const Vector3d& Bpc = XSbc;
  const Vector3d& Bqc = XEbc;
  
  const Vector3d Bq_Bp = Bqc-Bpc;
  const double& Bq_Bp_x = Bq_Bp[0];
  const double& Bq_Bp_y = Bq_Bp[1];
  const double& Bq_Bp_z = Bq_Bp[2];
  
//  const Vector3d P_Bp = (Pc-Bpc).normalized();
//  const double& P_Bp_x = P_Bp[0];
//  const double& P_Bp_y = P_Bp[1];
//  const double& P_Bp_z = P_Bp[2];
//  
//  const Vector3d Q_Bq = (Qc-Bqc).normalized();
//  const double& Q_Bq_x = Q_Bq[0];
//  const double& Q_Bq_y = Q_Bq[1];
//  const double& Q_Bq_z = Q_Bq[2];  
  
  const Vector3d VP = (Pc-Bpc).cross(Pc-Bqc).normalized()*lineLenghtInv; // ((P-Bp) X (P-Bq)).normalized()/|Bq-Bp|
  const double& VPx = VP[0];
  const double& VPy = VP[1];
  const double& VPz = VP[2];
  
  const Vector3d VQ = (Qc-Bpc).cross(Qc-Bqc).normalized()*lineLenghtInv; // ((Q-Bp) X (Q-Bq)).normalized()/|Bq-Bp|
  const double& VQx = VQ[0];
  const double& VQy = VQ[1];
  const double& VQz = VQ[2];  
  
// JdB_dXc_P = Jd3D_dXc_P = VP'*Bq_Bp_skew   
// JdB_dXc_P = [ + Bq_Bp_z*VPy - Bq_Bp_y*VPz, 
//               - Bq_Bp_z*VPx + Bq_Bp_x*VPz, 
//               + Bq_Bp_y*VPx - Bq_Bp_x*VPy]
  const double jprex_P =  + Bq_Bp_z*VPy - Bq_Bp_y*VPz;
  const double jprey_P =  - Bq_Bp_z*VPx + Bq_Bp_x*VPz;
  const double jprez_P =  + Bq_Bp_y*VPx - Bq_Bp_x*VPy;  
  
// JdB_dXc_Q = Jd3D_dXc_Q = VQ'*Bq_Bp_skew 
// JdB_dXc_Q = [  + Bq_Bp_z*VQy - Bq_Bp_y*VQz, 
//                - Bq_Bp_z*VQx + Bq_Bp_x*VQz, 
//                + Bq_Bp_y*VQx - Bq_Bp_x*VQy]  
  const double jprex_Q =  + Bq_Bp_z*VQy - Bq_Bp_y*VQz;
  const double jprey_Q =  - Bq_Bp_z*VQx + Bq_Bp_x*VQz;
  const double jprez_Q =  + Bq_Bp_y*VQx - Bq_Bp_x*VQy;     
  
       
/// < N.B.: we take the following Jacobian Jj as is without inverting the sign (we are dealing with a constraint h=0)
//[ - fy*l2 - y1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//    fx*l1 + x1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//   (fy*l2*x1)/z1 - (fx*l1*y1)/z1, 
//   (fx*l1)/z1, 
//   (fy*l2)/z1, 
// - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
//        
//[ - fy*l2 - y2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//    fx*l1 + x2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//   (fy*l2*x2)/z2 - (fx*l1*y2)/z2, 
//   (fx*l1)/z2, 
//   (fy*l2)/z2, 
// - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
          
  const double term1 = ( (fxl1*x1)+ (fyl2*y1) )*invz1_2;
     
  _jacobianOplusXi(0,0) = -fyl2 - y1*(term1);
  _jacobianOplusXi(0,1) =  fxl1 + x1*(term1);
  _jacobianOplusXi(0,2) = (fyl2*x1)*invz1 - (fxl1*y1)*invz1;
  _jacobianOplusXi(0,3) = (fxl1)*invz1;
  _jacobianOplusXi(0,4) = (fyl2)*invz1; 
  _jacobianOplusXi(0,5) = -(fxl1*x1)*invz1_2 - (fyl2*y1)*invz1_2;
  
  const double term2 = ( (fxl1*x2) + (fyl2*y2) )*invz2_2;
    
  _jacobianOplusXi(1,0) = -fyl2 - y2*(term2);
  _jacobianOplusXi(1,1) =  fxl1 + x2*(term2); 
  _jacobianOplusXi(1,2) = (fyl2*x2)*invz2 - (fxl1*y2)*invz2;
  _jacobianOplusXi(1,3) = (fxl1)*invz2; 
  _jacobianOplusXi(1,4) = (fyl2)*invz2;
  _jacobianOplusXi(1,5) = -(fxl1*x2)*invz2_2 - (fyl2*y2)*invz2_2;
 

// JdB_P = JdB_dXc_P_opq'*GTildePc          
// JdB_P = [ jprez_P*y1c - jprey_P*z1c, 
//           jprex_P*z1c - jprez_P*x1c, 
//           jprey_P*x1c - jprex_P*y1c, 
//           jprex_P, 
//           jprey_P, 
//           jprez_P]
  _jacobianOplusXi(2,0) = jprez_P*y1 - jprey_P*z1;
  _jacobianOplusXi(2,1) = jprex_P*z1 - jprez_P*x1;
  _jacobianOplusXi(2,2) = jprey_P*x1 - jprex_P*y1;
  _jacobianOplusXi(2,3) = jprex_P;
  _jacobianOplusXi(2,4) = jprey_P;
  _jacobianOplusXi(2,5) = jprez_P;
          

// JdB_Q = JdB_dXc_Q_opq'*GTildeQc
// JdB_Q = [ jprez_Q*y2c - jprey_Q*z2c, 
//           jprex_Q*z2c - jprez_Q*x2c, 
//           jprey_Q*x2c - jprex_Q*y2c, 
//           jprex_Q, 
//           jprey_Q, 
//           jprez_Q]
  _jacobianOplusXi(3,0) = jprez_Q*y2 - jprey_Q*z2 ;
  _jacobianOplusXi(3,1) = jprex_Q*z2 - jprez_Q*x2 ;
  _jacobianOplusXi(3,2) = jprey_Q*x2 - jprex_Q*y2 ;
  _jacobianOplusXi(3,3) = jprex_Q;
  _jacobianOplusXi(3,4) = jprey_Q;
  _jacobianOplusXi(3,5) = jprez_Q;
}
#endif

// -- -- -- --

EdgeSE3ProjectLine::EdgeSE3ProjectLine() : BaseBinaryEdge<2, Vector3d, VertexSBALine, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectLine::read(std::istream& is){
  for (int i=0; i<3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectLine::write(std::ostream& os) const {

  for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

#if USE_ANALYTIC_JACS_FULL_MONO
void EdgeSE3ProjectLine::linearizeOplus() {
  const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const SE3Quat& T = vj->estimate();
  const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
  const Vector3d xyz_trans_s = T.map(vi->estimate().head<3>());
  const Vector3d xyz_trans_e = T.map(vi->estimate().tail<3>());
  
  const double& l1 = _measurement[0];
  const double& l2 = _measurement[1];
  //const double& l3 = _measurement[2];
  
  const double& x1 = xyz_trans_s[0];
  const double& y1 = xyz_trans_s[1];
  const double invz1 = 1.0/xyz_trans_s[2];
  const double invz1_2 = invz1*invz1;
  
  const double& x2 = xyz_trans_e[0];
  const double& y2 = xyz_trans_e[1];
  const double invz2 = 1.0/xyz_trans_e[2];
  const double invz2_2 = invz2*invz2;
  
  const double fxl1 = fx*l1;
  const double fyl2 = fy*l2;
  
/// < N.B.: we take the following Jacobian Ji as is without inverting the sign as below  (we are dealing with a constraint h=0)
// Jlineproj1 = [ (fx*l1)/z1, (fy*l2)/z1, - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
// Jlineproj2 = [ (fx*l1)/z2, (fy*l2)/z2, - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
          
  Matrix<double,2,3> Jproj;
  Jproj(0,0) = (fxl1)*invz1;
  Jproj(0,1) = (fyl2)*invz1;
  Jproj(0,2) = - ((fxl1*x1)+(fyl2*y1))*invz1_2;       
   
  Jproj(1,0) = (fxl1)*invz2;
  Jproj(1,1) = (fyl2)*invz2;
  Jproj(1,2) = - ((fxl1*x2)+(fyl2*y2))*invz2_2; 
  
  Matrix<double,2,3> Jline;
  Jline = Jproj * T.rotation().toRotationMatrix();
  
  _jacobianOplusXi(0,0) = Jline(0,0);
  _jacobianOplusXi(0,1) = Jline(0,1);
  _jacobianOplusXi(0,2) = Jline(0,2);
  _jacobianOplusXi(0,3) = 0;
  _jacobianOplusXi(0,4) = 0;         
  _jacobianOplusXi(0,5) = 0;          
  
  _jacobianOplusXi(1,0) = 0;
  _jacobianOplusXi(1,1) = 0;         
  _jacobianOplusXi(1,2) = 0;        
  _jacobianOplusXi(1,3) = Jline(1,0);
  _jacobianOplusXi(1,4) = Jline(1,1);
  _jacobianOplusXi(1,5) = Jline(1,2);

  
/// < N.B.: we take the following Jacobian Jj as is without inverting the sign (we are dealing with a constraint h=0)
//[ - fy*l2 - y1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//    fx*l1 + x1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//   (fy*l2*x1)/z1 - (fx*l1*y1)/z1, 
//   (fx*l1)/z1, 
//   (fy*l2)/z1, 
// - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
//        
//[ - fy*l2 - y2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//    fx*l1 + x2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//   (fy*l2*x2)/z2 - (fx*l1*y2)/z2, 
//   (fx*l1)/z2, 
//   (fy*l2)/z2, 
// - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
          
  const double term1 = ( (fxl1*x1)+ (fyl2*y1) )*invz1_2;
     
  _jacobianOplusXj(0,0) = -fyl2 - y1*(term1);
  _jacobianOplusXj(0,1) =  fxl1 + x1*(term1);
  _jacobianOplusXj(0,2) = (fyl2*x1)*invz1 - (fxl1*y1)*invz1;
  _jacobianOplusXj(0,3) = (fxl1)*invz1;
  _jacobianOplusXj(0,4) = (fyl2)*invz1; 
  _jacobianOplusXj(0,5) = -(fxl1*x1)*invz1_2 - (fyl2*y1)*invz1_2;
  
  const double term2 = ( (fxl1*x2) + (fyl2*y2) )*invz2_2;
    
  _jacobianOplusXj(1,0) = -fyl2 - y2*(term2);
  _jacobianOplusXj(1,1) =  fxl1 + x2*(term2); 
  _jacobianOplusXj(1,2) = (fyl2*x2)*invz2 - (fxl1*y2)*invz2;
  _jacobianOplusXj(1,3) = (fxl1)*invz2; 
  _jacobianOplusXj(1,4) = (fyl2)*invz2;
  _jacobianOplusXj(1,5) = -(fxl1*x2)*invz2_2 - (fyl2*y2)*invz2_2;
 
}
#endif
  

Vector2d EdgeSE3ProjectLine::camProject(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector2d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  return res;
}



/// < < < 


bool EdgeLinePrior::read(std::istream& is){
  for (int i=0; i<6; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeLinePrior::write(std::ostream& os) const {

  for (int i=0; i<6; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeLinePrior::linearizeOplus() {
  
 
  _jacobianOplusXi = -Matrix6d::Identity();
 
}


// -- -- -- --

EdgeSE3ProjectStereoLine::EdgeSE3ProjectStereoLine() {
    mu = 1;
    lineLenghtInv = 1;
}

bool EdgeSE3ProjectStereoLine::read(std::istream& is){
  for (int i=0; i<3; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectStereoLine::write(std::ostream& os) const {

  for (int i=0; i<3; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


Vector2d EdgeSE3ProjectStereoLine::camProject(const Vector3d & trans_xyz) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector2d res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  return res;
}

Vector3d EdgeSE3ProjectStereoLine::camBackProject(const Vector2d & p_uv, const double depth ) const{
  Vector3d res;
  res[0] = (p_uv[0]-cx)*depth/fx;
  res[1] = (p_uv[1]-cy)*depth/fy; 
  res[2] = depth;
  return res;
}

#if USE_ANALYTIC_JACS_FULL_STEREO
void EdgeSE3ProjectStereoLine::linearizeOplus() {
  const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const SE3Quat& T = vj->estimate();
  const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
  const Vector3d xyz_trans_s = T.map(vi->estimate().head(3));
  const Vector3d xyz_trans_e = T.map(vi->estimate().tail(3));
  
  const double& l1 = _measurement[0];
  const double& l2 = _measurement[1];
  //const double& l3 = _measurement[2];
  
  const double& x1 = xyz_trans_s[0];
  const double& y1 = xyz_trans_s[1];
  const double& z1 = xyz_trans_s[2];
  const double invz1 = 1.0/xyz_trans_s[2];
  const double invz1_2 = invz1*invz1;
  
  const double& x2 = xyz_trans_e[0];
  const double& y2 = xyz_trans_e[1];
  const double& z2 = xyz_trans_e[2];  
  const double invz2 = 1.0/xyz_trans_e[2];
  const double invz2_2 = invz2*invz2;
  
  const double fxl1 = fx*l1;
  const double fyl2 = fy*l2;
  
  const Vector3d& Pc = xyz_trans_s;
  const Vector3d& Qc = xyz_trans_e;
  
  const Vector3d& Bpc = XSbc;
  const Vector3d& Bqc = XEbc;
  
  const Vector3d Bq_Bp = Bqc-Bpc;
  const double& Bq_Bp_x = Bq_Bp[0];
  const double& Bq_Bp_y = Bq_Bp[1];
  const double& Bq_Bp_z = Bq_Bp[2];
  
  const Vector3d P_Bp = (Pc-Bpc).normalized();
  const double& P_Bp_x = P_Bp[0];
  const double& P_Bp_y = P_Bp[1];
  const double& P_Bp_z = P_Bp[2];
  
  const Vector3d Q_Bq = (Qc-Bqc).normalized();
  const double& Q_Bq_x = Q_Bq[0];
  const double& Q_Bq_y = Q_Bq[1];
  const double& Q_Bq_z = Q_Bq[2];  
  
  const Vector3d VP = (Pc-Bpc).cross(Pc-Bqc).normalized()*lineLenghtInv; // ((P-Bp) X (P-Bq)).normalized()/|Bq-Bp|
  const double& VPx = VP[0];
  const double& VPy = VP[1];
  const double& VPz = VP[2];
  
  const Vector3d VQ = (Qc-Bpc).cross(Qc-Bqc).normalized()*lineLenghtInv; // ((Q-Bp) X (Q-Bq)).normalized()/|Bq-Bp|
  const double& VQx = VQ[0];
  const double& VQy = VQ[1];
  const double& VQz = VQ[2];  
  
// JdB_dXc_P = Jd3D_dXc_P + JdP_dXc_P = VP'*Bq_Bp_skew + mu * P_Bp'  
// JdB_dXc_P = [ mu * P_Bp_x + Bq_Bp_z*VPy - Bq_Bp_y*VPz, 
//               mu * P_Bp_y - Bq_Bp_z*VPx + Bq_Bp_x*VPz, 
//               mu * P_Bp_z + Bq_Bp_y*VPx - Bq_Bp_x*VPy]
  const double jprex_P = mu * P_Bp_x + Bq_Bp_z*VPy - Bq_Bp_y*VPz;
  const double jprey_P = mu * P_Bp_y - Bq_Bp_z*VPx + Bq_Bp_x*VPz;
  const double jprez_P = mu * P_Bp_z + Bq_Bp_y*VPx - Bq_Bp_x*VPy;  
  
// JdB_dXc_Q = Jd3D_dXc_Q + JdP_dXc_Q = VQ'*Bq_Bp_skew + mu * Q_Bq'  
// JdB_dXc_Q = [ mu * Q_Bq_x + Bq_Bp_z*VQy - Bq_Bp_y*VQz, 
//               mu * Q_Bq_y - Bq_Bp_z*VQx + Bq_Bp_x*VQz, 
//               mu * Q_Bq_z + Bq_Bp_y*VQx - Bq_Bp_x*VQy]  
  const double jprex_Q = mu * Q_Bq_x + Bq_Bp_z*VQy - Bq_Bp_y*VQz;
  const double jprey_Q = mu * Q_Bq_y - Bq_Bp_z*VQx + Bq_Bp_x*VQz;
  const double jprez_Q = mu * Q_Bq_z + Bq_Bp_y*VQx - Bq_Bp_x*VQy;   

  
/// < N.B.: we take the following Jacobian Ji as is without inverting the sign as below  (we are dealing with a constraint h=0)
// Jlineproj1 = [ (fx*l1)/z1, (fy*l2)/z1, - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
// Jlineproj2 = [ (fx*l1)/z2, (fy*l2)/z2, - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
          
  Matrix<double,4,3> Jproj;
  Jproj(0,0) = (fxl1)*invz1;
  Jproj(0,1) = (fyl2)*invz1;
  Jproj(0,2) = - ((fxl1*x1)+(fyl2*y1))*invz1_2;       
   
  Jproj(1,0) = (fxl1)*invz2;
  Jproj(1,1) = (fyl2)*invz2;
  Jproj(1,2) = - ((fxl1*x2)+(fyl2*y2))*invz2_2; 

  Jproj(2,0) = jprex_P;
  Jproj(2,1) = jprey_P;
  Jproj(2,2) = jprez_P;  
  
  Jproj(3,0) = jprex_Q;
  Jproj(3,1) = jprey_Q;
  Jproj(3,2) = jprez_Q;            
  
  Matrix<double,4,3> Jline;
  Jline = Jproj * T.rotation().toRotationMatrix();
  
  _jacobianOplusXi(0,0) = Jline(0,0);
  _jacobianOplusXi(0,1) = Jline(0,1);
  _jacobianOplusXi(0,2) = Jline(0,2);
  _jacobianOplusXi(0,3) = 0;
  _jacobianOplusXi(0,4) = 0;         
  _jacobianOplusXi(0,5) = 0;          
  
  _jacobianOplusXi(1,0) = 0;
  _jacobianOplusXi(1,1) = 0;         
  _jacobianOplusXi(1,2) = 0;        
  _jacobianOplusXi(1,3) = Jline(1,0);
  _jacobianOplusXi(1,4) = Jline(1,1);
  _jacobianOplusXi(1,5) = Jline(1,2);
  
  _jacobianOplusXi(2,0) = Jline(2,0);
  _jacobianOplusXi(2,1) = Jline(2,1);        
  _jacobianOplusXi(2,2) = Jline(2,2);        
  _jacobianOplusXi(2,3) = 0;
  _jacobianOplusXi(2,4) = 0;
  _jacobianOplusXi(2,5) = 0; 
          
  _jacobianOplusXi(3,0) = 0;
  _jacobianOplusXi(3,1) = 0;        
  _jacobianOplusXi(3,2) = 0;        
  _jacobianOplusXi(3,3) = Jline(3,0);
  _jacobianOplusXi(3,4) = Jline(3,1);
  _jacobianOplusXi(3,5) = Jline(3,2);            


/// < N.B.: we take the following Jacobian Jj as is without inverting the sign (we are dealing with a constraint h=0)
//[ - fy*l2 - y1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//    fx*l1 + x1*((fx*l1*x1)/z1^2 + (fy*l2*y1)/z1^2), 
//   (fy*l2*x1)/z1 - (fx*l1*y1)/z1, 
//   (fx*l1)/z1, 
//   (fy*l2)/z1, 
// - (fx*l1*x1)/z1^2 - (fy*l2*y1)/z1^2]
//        
//[ - fy*l2 - y2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//    fx*l1 + x2*((fx*l1*x2)/z2^2 + (fy*l2*y2)/z2^2), 
//   (fy*l2*x2)/z2 - (fx*l1*y2)/z2, 
//   (fx*l1)/z2, 
//   (fy*l2)/z2, 
// - (fx*l1*x2)/z2^2 - (fy*l2*y2)/z2^2]
          
  const double term1 = ( (fxl1*x1)+ (fyl2*y1) )*invz1_2;
     
  _jacobianOplusXj(0,0) = -fyl2 - y1*(term1);
  _jacobianOplusXj(0,1) =  fxl1 + x1*(term1);
  _jacobianOplusXj(0,2) = (fyl2*x1)*invz1 - (fxl1*y1)*invz1;
  _jacobianOplusXj(0,3) = (fxl1)*invz1;
  _jacobianOplusXj(0,4) = (fyl2)*invz1; 
  _jacobianOplusXj(0,5) = -(fxl1*x1)*invz1_2 - (fyl2*y1)*invz1_2;
  
  const double term2 = ( (fxl1*x2) + (fyl2*y2) )*invz2_2;
    
  _jacobianOplusXj(1,0) = -fyl2 - y2*(term2);
  _jacobianOplusXj(1,1) =  fxl1 + x2*(term2); 
  _jacobianOplusXj(1,2) = (fyl2*x2)*invz2 - (fxl1*y2)*invz2;
  _jacobianOplusXj(1,3) = (fxl1)*invz2; 
  _jacobianOplusXj(1,4) = (fyl2)*invz2;
  _jacobianOplusXj(1,5) = -(fxl1*x2)*invz2_2 - (fyl2*y2)*invz2_2;
 

// JdB_P = JdB_dXc_P_opq'*GTildePc          
// JdB_P = [ jprez_P*y1c - jprey_P*z1c, 
//           jprex_P*z1c - jprez_P*x1c, 
//           jprey_P*x1c - jprex_P*y1c, 
//           jprex_P, 
//           jprey_P, 
//           jprez_P]
  _jacobianOplusXj(2,0) = jprez_P*y1 - jprey_P*z1;
  _jacobianOplusXj(2,1) = jprex_P*z1 - jprez_P*x1;
  _jacobianOplusXj(2,2) = jprey_P*x1 - jprex_P*y1;
  _jacobianOplusXj(2,3) = jprex_P;
  _jacobianOplusXj(2,4) = jprey_P;
  _jacobianOplusXj(2,5) = jprez_P;
          
// JdB_Q = JdB_dXc_Q_opq'*GTildeQc
// JdB_Q = [ jprez_Q*y2c - jprey_Q*z2c, 
//           jprex_Q*z2c - jprez_Q*x2c, 
//           jprey_Q*x2c - jprex_Q*y2c, 
//           jprex_Q, 
//           jprey_Q, 
//           jprez_Q]
  _jacobianOplusXj(3,0) = jprez_Q*y2 - jprey_Q*z2;
  _jacobianOplusXj(3,1) = jprex_Q*z2 - jprez_Q*x2;
  _jacobianOplusXj(3,2) = jprey_Q*x2 - jprex_Q*y2;
  _jacobianOplusXj(3,3) = jprex_Q;
  _jacobianOplusXj(3,4) = jprey_Q;
  _jacobianOplusXj(3,5) = jprez_Q;
}
#endif

} // end namespace
