/*
 * This file is part of PLVS.

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

#pragma once

#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/core/eigen_types.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sba/types_sba.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/se3_ops.h"
#include "Thirdparty/g2o/g2o/types/se3quat.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#endif 

#include "g2o/types_sba_line.h"

#include <Eigen/Geometry>
#include <include/CameraModels/GeometricCamera.h>


namespace PLVS2 {


#define USE_ANALYTIC_JACS 1

#define USE_ANALYTIC_JACS_ONLY_POSE         (1 && USE_ANALYTIC_JACS)
#define USE_ANALYTIC_JACS_ONLY_POSE_TO_BODY (1 && USE_ANALYTIC_JACS)

#define USE_ANALYTIC_JACS_FULL         (1 && USE_ANALYTIC_JACS)
#define USE_ANALYTIC_JACS_FULL_TO_BODY (1 && USE_ANALYTIC_JACS)

#define NEW_LINE_ENDPOINTS_CHECKING 1

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class  EdgeSE3ProjectLineOnlyPose: public g2o::BaseUnaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, g2o::VertexSE3Expmap>{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectLineOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::SE3Quat& T = vi->estimate();
    //_measurement[0,1,2] comes as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement.dot( pCamera->projectLinear3(T.map(XSw)) );
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement.dot( pCamera->projectLinear3(T.map(XEw)) );
  }

  bool isStartDepthPositive() {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    return (vi->estimate().map(XSw))(2)>0.0;
  }

    bool isEndDepthPositive() {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    return (vi->estimate().map(XEw))(2)>0.0;
  }
    
  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]); 
    const g2o::SE3Quat& T = vi->estimate();    
    const Eigen::Vector3d mapLineS = T.map(XSw); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(XEw); // map Q w.r.t. camera frame
    projS = pCamera->projectLinear(mapLineS); 
    projE = pCamera->projectLinear(mapLineE); 
  }   

  #if USE_ANALYTIC_JACS_ONLY_POSE
  virtual void linearizeOplus();
  #endif

  Eigen::Vector3d XSw; // start point 
  Eigen::Vector3d XEw; // end point 
  GeometricCamera* pCamera=nullptr;
}; 

// -- -- -- -- 

class  EdgeSE3ProjectLineOnlyPoseToBody: public g2o::BaseUnaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectLineOnlyPoseToBody(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vi->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;
    //_measurement[0,1,2] comes as a right line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement.dot( pCamera->projectLinear3( Trw.map(XSw)) );
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement.dot( pCamera->projectLinear3( Trw.map(XEw)) );
  }

  bool isStartDepthPositive() {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vi->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;    
    return (Trw.map(XSw))(2)>0.0;
  }
  
   bool isEndDepthPositive() {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vi->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;      
    return (Trw.map(XEw))(2)>0.0;
  }
   
  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const g2o::VertexSE3Expmap* vi = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]); 
    const g2o::SE3Quat& Tlw = vi->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;     
    projS = pCamera->projectLinear(Trw.map(XSw)); 
    projE = pCamera->projectLinear(Trw.map(XEw)); 
  }   

#if USE_ANALYTIC_JACS_ONLY_POSE_TO_BODY
  virtual void linearizeOplus();
#endif

  Eigen::Vector3d XSw; // start point 
  Eigen::Vector3d XEw; // end point 
  GeometricCamera* pCamera=nullptr;

  g2o::SE3Quat mTrl;
}; 



class  EdgeSE3ProjectLine: public  g2o::BaseBinaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, g2o::VertexSBALine, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

  EdgeSE3ProjectLine();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]); 
    const g2o::SE3Quat& T = vj->estimate();    
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    const Eigen::Vector2d projS = pCamera->projectLinear(mapLineS); 
    const Eigen::Vector2d projE = pCamera->projectLinear(mapLineE); 
    
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
  }

  bool isStartDepthPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().head(3)))(2)>0.0;
  }
  
  bool isEndDepthPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().tail(3)))(2)>0.0;
  }
  
  bool areDepthsPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]); 
    const g2o::SE3Quat& T = vj->estimate();   
    const Vector6d& line = vi->estimate();
    return ((T.map(line.head(3)))(2)>0.0)&&((T.map(line.tail(3)))(2)>0.0);
  }
  
  double depthStart(){
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().head(3)))(2);
  }
  
   double depthEnd(){
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().tail(3)))(2);
  }

  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi   = static_cast<const g2o::VertexSBALine*>(_vertices[0]); 
    const g2o::SE3Quat& T = vj->estimate();    
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    projS = pCamera->projectLinear(mapLineS); 
    projE = pCamera->projectLinear(mapLineE); 
  }
   
#if USE_ANALYTIC_JACS_FULL 
  virtual void linearizeOplus();
#endif

  GeometricCamera* pCamera=nullptr;

};



class  EdgeSE3ProjectLineToBody: public  g2o::BaseBinaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, g2o::VertexSBALine, g2o::VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

  EdgeSE3ProjectLineToBody();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);    
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;    
    const Vector6d& mapLine = vi->estimate(); // (Pw,Qw)
    const Eigen::Vector3d mapLineS = Trw.map(mapLine.head(3)); // map P w.r.t. right camera frame
    const Eigen::Vector3d mapLineE = Trw.map(mapLine.tail(3)); // map Q w.r.t. right camera frame
    const Eigen::Vector2d projS = pCamera->projectLinear(mapLineS); 
    const Eigen::Vector2d projE = pCamera->projectLinear(mapLineE); 
    
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
  }

  bool isStartDepthPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;       
    return (Trw.map(vi->estimate().head(3)))(2)>0.0;
  }
  
  bool isEndDepthPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;      
    return (Trw.map(vi->estimate().tail(3)))(2)>0.0;
  }
  
  bool areDepthsPositive() {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]); 
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;     
    const Vector6d& line = vi->estimate();     
    return ((Trw.map(line.head(3)))(2)>0.0)&&((Trw.map(line.tail(3)))(2)>0.0);
  }
  
  double depthStart(){
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;      
    return (Trw.map(vi->estimate().head(3)))(2);
  }
  
   double depthEnd(){
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
    const g2o::VertexSBALine* vi = static_cast<const g2o::VertexSBALine*>(_vertices[0]);
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;          
    return (Trw.map(vi->estimate().tail(3)))(2);
  }

  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const g2o::VertexSE3Expmap* vj = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]); 
    const g2o::VertexSBALine* vi   = static_cast<const g2o::VertexSBALine*>(_vertices[0]); 
    const g2o::SE3Quat& Tlw = vj->estimate();
    const g2o::SE3Quat Trw = mTrl * Tlw;          
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = Trw.map(mapLine.head(3)); // map P w.r.t. right camera frame
    const Eigen::Vector3d mapLineE = Trw.map(mapLine.tail(3)); // map Q w.r.t. right camera frame
    projS = pCamera->projectLinear(mapLineS); 
    projE = pCamera->projectLinear(mapLineE); 
  }
   
#if USE_ANALYTIC_JACS_FULL_TO_BODY 
  virtual void linearizeOplus();
#endif

  GeometricCamera* pCamera=nullptr;
  g2o::SE3Quat mTrl;
};

}

