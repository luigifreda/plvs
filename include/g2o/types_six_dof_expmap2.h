// This file is part of PLVS
// Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Created by Luigi Freda (2017)
// Added EdgeSE3ProjectLineOnlyPose (unary edge to optimize only camera pose )
// Added EdgeSE3ProjectStereoLineOnlyPose (unary edge to optimize only camera pose )
// Added EdgeSE3ProjectLine (binary edge to optimize camera pose and line representation)
// Added EdgeSE3ProjectStereoLine (binary edge to optimize camera pose and line representation)


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

namespace g2o {
namespace types_six_dof_expmap {
void init();
}

typedef Eigen::Matrix<double, 6, 6> Matrix6d;



class  EdgeRgbdSE3ProjectXYZ: public  BaseBinaryEdge<3, Eigen::Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeRgbdSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - camProject(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }


  virtual void linearizeOplus();

  Eigen::Vector3d camProject(const Eigen::Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
};


class  EdgeRgbdSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeRgbdSE3ProjectXYZOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d obs(_measurement);
    _error = obs - camProject(v1->estimate().map(Xw));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (v1->estimate().map(Xw))(2)>0.0;
  }


  virtual void linearizeOplus();

  Eigen::Vector3d camProject(const Eigen::Vector3d & trans_xyz) const;

  Eigen::Vector3d Xw;
  double fx, fy, cx, cy;
};


/// < < <

#define USE_ANALYTIC_JACS 1

#define USE_ANALYTIC_JACS_ONLY_POSE_MONO   (1 && USE_ANALYTIC_JACS)
#define USE_ANALYTIC_JACS_ONLY_POSE_STEREO (1 && USE_ANALYTIC_JACS)

#define USE_ANALYTIC_JACS_FULL_MONO   (1 && USE_ANALYTIC_JACS)
#define USE_ANALYTIC_JACS_FULL_STEREO (1 && USE_ANALYTIC_JACS)

#define NEW_LINE_ENDPOINTS_CHECKING 1


class  EdgeSE3ProjectLineOnlyPose: public  BaseUnaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectLineOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const SE3Quat& T = vi->estimate();
    //_measurement[0,1,2] comes as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement.dot( camProject(T.map(XSw)) );
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement.dot( camProject(T.map(XEw)) );
  }

  bool isStartDepthPositive() {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (vi->estimate().map(XSw))(2)>0.0;
  }
  
   bool isEndDepthPositive() {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return (vi->estimate().map(XEw))(2)>0.0;
  }
   
  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]); 
    const SE3Quat& T = vi->estimate();    
    const Eigen::Vector3d mapLineS = T.map(XSw); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(XEw); // map Q w.r.t. camera frame
    projS = camProject2d(mapLineS); 
    projE = camProject2d(mapLineE); 
  }   

#if USE_ANALYTIC_JACS_ONLY_POSE_MONO
  virtual void linearizeOplus();
#endif

  Eigen::Vector3d camProject(const Eigen::Vector3d & trans_xyz) const;
  Eigen::Vector2d camProject2d(const Eigen::Vector3d & trans_xyz) const;

  Eigen::Vector3d XSw; // start point 
  Eigen::Vector3d XEw; // end point 
  double fx, fy, cx, cy;
}; 

// -- -- -- -- 

typedef Eigen::Matrix<double, 5, 1> Vector5d;

class  EdgeSE3ProjectStereoLineOnlyPose: public  BaseUnaryEdge<4/*error size*/, Eigen::Matrix<double, 3, 1>/*meas type*/, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectStereoLineOnlyPose(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;
  
  void computeError()  {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const SE3Quat& T = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(XSw); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(XEw); // map Q w.r.t. camera frame
    const Eigen::Vector2d projS = camProject(mapLineS); 
    const Eigen::Vector2d projE = camProject(mapLineE); 
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
    
    const Eigen::Vector3d& backprojLineSb = XSbc;
    const Eigen::Vector3d& backprojLineEb = XEbc;
    
    // align 3D map-line to 3D detected-line
    _error[2] = (mapLineS - backprojLineSb).cross(mapLineS-backprojLineEb).norm() * lineLenghtInv; //  |(P-Bp) X (P-Bq)|/|Bq-Bp|
    _error[3] = (mapLineE - backprojLineSb).cross(mapLineE-backprojLineEb).norm() * lineLenghtInv; //  |(Q-Bp) X (Q-Bq)|/|Bq-Bp|
  }

  void getMapLineAndProjections(Eigen::Vector3d& mapS, Eigen::Vector3d& mapE, Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]); 
    const SE3Quat& T = vi->estimate();    
    mapS = T.map(XSw); // map P w.r.t. camera frame
    mapE = T.map(XEw); // map Q w.r.t. camera frame
    projS = camProject(mapS); 
    projE = camProject(mapE); 
  }   
  
#if USE_ANALYTIC_JACS_ONLY_POSE_STEREO
  virtual void linearizeOplus();
#endif

  Eigen::Vector2d camProject(const Eigen::Vector3d & trans_xyz) const;
  
  Eigen::Vector3d camBackProject(const Eigen::Vector2d & p_uv, const double depth ) const;
  
  
    // to be called after all the data have been filled up
  void init(){
    const VertexSE3Expmap* vi = static_cast<const VertexSE3Expmap*>(_vertices[0]); 
    const SE3Quat& T = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(XSw); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(XEw); // map Q w.r.t. camera frame
    
    // check if the end-points association is correct
#if !NEW_LINE_ENDPOINTS_CHECKING
    const double distSSd = (mapLineS-XSbc).norm();
    const double distSEd = (mapLineS-XEbc).norm();
    if(distSSd > distSEd)
    {
        //std::cout << "PO: swapping line endpoints" << std::endl;
        std::swap(XSbc,XEbc);
    }
#else    
    const Eigen::Vector3d mapLineSE = (mapLineS-mapLineE).normalized();
    const Eigen::Vector3d lineSE = (XSbc-XEbc).normalized();    
    if(mapLineSE.dot(lineSE) < 0)
    {
        //std::cout << "BA: swapping line endpoints" << std::endl;
        std::swap(XSbc,XEbc);
    }        
#endif      
  }
  
  double computeSquared3DError()
  {
        // compute full 3D error without information matrix    
      return (_error[2]*_error[2] + _error[3]*_error[3]);
  }

  Eigen::Vector3d XSw;  // start point w.r.t. world frame 
  Eigen::Vector3d XEw;  // end point w.r.t. world frame 
  Eigen::Vector3d XSbc; // backprojected start point w.r.t. camera  frame 
  Eigen::Vector3d XEbc; // backprojected end point w.r.t camera frame 
  double lineLenghtInv; // inverse of backprojected line length 
  double fx, fy, cx, cy;
};



// -- -- -- -- 



class  EdgeLinePrior: public  BaseUnaryEdge<6/*error size*/, Vector6d/*meas type*/, VertexSBALine>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeLinePrior(){}

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSBALine* v1 = static_cast<const VertexSBALine*>(_vertices[0]);
    const Vector6d& line = v1->estimate();
    const Eigen::Vector3d lineStart = line.head(3);
    const Eigen::Vector3d lineEnd = line.tail(3);
    _error[0] = _measurement[0] - lineStart[0];
    _error[1] = _measurement[1] - lineStart[1];
    _error[2] = _measurement[2] - lineStart[2];
    _error[3] = _measurement[3] - lineEnd[0];
    _error[4] = _measurement[4] - lineEnd[1];
    _error[5] = _measurement[5] - lineEnd[2];
  }

  virtual void linearizeOplus();

}; 


// -- -- -- -- 

class  EdgeSE3ProjectLine: public  BaseBinaryEdge<2/*error size*/, Eigen::Vector3d/*meas type*/, VertexSBALine, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  

  EdgeSE3ProjectLine();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi   = static_cast<const VertexSBALine*>(_vertices[0]); 
    const SE3Quat& T = vj->estimate();    
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    const Eigen::Vector2d projS = camProject(mapLineS); 
    const Eigen::Vector2d projE = camProject(mapLineE); 
    
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
  }

  bool isStartDepthPositive() {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().head(3)))(2)>0.0;
  }
  
  bool isEndDepthPositive() {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().tail(3)))(2)>0.0;
  }
  
  bool areDepthsPositive() {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]); 
    const SE3Quat& T = vj->estimate();   
    const Vector6d& line = vi->estimate();
    return ((T.map(line.head(3)))(2)>0.0)&&((T.map(line.tail(3)))(2)>0.0);
  }
  
  double depthStart(){
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().head(3)))(2);
  }
  
   double depthEnd(){
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);
    return (vj->estimate().map(vi->estimate().tail(3)))(2);
  }

  void getMapLineProjections(Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi   = static_cast<const VertexSBALine*>(_vertices[0]); 
    const SE3Quat& T = vj->estimate();    
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    projS = camProject(mapLineS); 
    projE = camProject(mapLineE); 
  }
   
#if USE_ANALYTIC_JACS_FULL_MONO 
  virtual void linearizeOplus();
#endif

  Eigen::Vector2d camProject(const Eigen::Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;

};


// -- -- -- -- 


class  EdgeSE3ProjectStereoLine: public  BaseBinaryEdge<4/*error size*/, Eigen::Vector3d/*meas type*/, VertexSBALine, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectStereoLine();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);   
    const SE3Quat& T = vj->estimate();
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    const Eigen::Vector2d projS = camProject(mapLineS); 
    const Eigen::Vector2d projE = camProject(mapLineE); 
    
    // _measurement[0,1,2] come as a line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
    // constraint: 0 = l1*u1 + l2*v1 + l3  (we can consider a fake "observation" always equal to zero)
    _error[0] = _measurement[0]*projS[0] + _measurement[1]*projS[1] + _measurement[2];
    // constraint: 0 = l1*u2 + l2*v2 + l3  (we can consider a fake "observation" always equal to zero)
    _error[1] = _measurement[0]*projE[0] + _measurement[1]*projE[1] + _measurement[2];
    
    const Eigen::Vector3d& backprojLineSb = XSbc;
    const Eigen::Vector3d& backprojLineEb = XEbc;
        
    // align each point of 3D map-line to 3D detected-line (by moving the camera frame and adjusting 3D map-line points)
    const Eigen::Vector3d S_Sb = mapLineS - backprojLineSb; // P-Bp
    const Eigen::Vector3d E_Eb = mapLineE - backprojLineEb; // Q-Bq
    _error[2] = S_Sb.cross(mapLineS - backprojLineEb).norm() * lineLenghtInv + mu * S_Sb.norm();   //  |(P-Bp) X (P-Bq)|/|Bq-Bp| + mu * |P-Bp|
    _error[3] = (mapLineE - backprojLineSb).cross(E_Eb).norm() * lineLenghtInv + mu * E_Eb.norm(); //  |(Q-Bp) X (Q-Bq)|/|Bq-Bp| + mu * |Q-Bq|   
  }
  
  // to be called after all the data have been filled up
  void init(){
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);   
    const SE3Quat& T = vj->estimate();
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    
    // check if the end-points association is correct
#if !NEW_LINE_ENDPOINTS_CHECKING 
    const double distS_Sb = (mapLineS-XSbc).norm();
    const double distS_Eb = (mapLineS-XEbc).norm();    
    if(distS_Sb > distS_Eb)
    {
        //std::cout << "BA: swapping line endpoints" << std::endl;
        std::swap(XSbc,XEbc);
    }
#else
    const Eigen::Vector3d mapLineSE = (mapLineS-mapLineE).normalized();
    const Eigen::Vector3d lineSE = (XSbc-XEbc).normalized();    
    if(mapLineSE.dot(lineSE) < 0)
    {
        //std::cout << "BA: swapping line endpoints" << std::endl;
        std::swap(XSbc,XEbc);
    }    
#endif 
      
  }
  
  bool areDepthsPositive() {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]); 
    const SE3Quat& T = vj->estimate();   
    const Vector6d& line = vi->estimate();
    return ((T.map(line.head(3)))(2)>0.0)&&((T.map(line.tail(3)))(2)>0.0);
  }  
  
  
  void getMapLineAndProjections(Eigen::Vector3d& mapS, Eigen::Vector3d& mapE, Eigen::Vector2d& projS, Eigen::Vector2d& projE)  {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi   = static_cast<const VertexSBALine*>(_vertices[0]); 
    const SE3Quat& T = vj->estimate();    
    const Vector6d& mapLine = vi->estimate();
    mapS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    mapE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame
    projS = camProject(mapS); 
    projE = camProject(mapE); 
  }
    
  // compute just line-alignment error (do not consider the constraints/regularizers on the line end-points)  
  double zeroMuChi2()  
  {
    const double mu_current = mu;
    mu = 0; 
    computeError();  
    const double res = _error.dot(information()*_error);
    mu = mu_current;    
    return res;
  }    
  
  // compute total squared 3D error 
  double computeSquared3DError()
  {
    // compute full 3D error with used mu and without information matrix
    computeError(); 
    return (_error[2]*_error[2] + _error[3]*_error[3]);
  }  
  
  // compute total squared end-points deviations 
  double computeSquaredEndPointsDeviations()  
  {
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]); 
    const VertexSBALine* vi = static_cast<const VertexSBALine*>(_vertices[0]);   
    const SE3Quat& T = vj->estimate();
    const Vector6d& mapLine = vi->estimate();
    const Eigen::Vector3d mapLineS = T.map(mapLine.head(3)); // map P w.r.t. camera frame
    const Eigen::Vector3d mapLineE = T.map(mapLine.tail(3)); // map Q w.r.t. camera frame

    const Eigen::Vector3d& backprojLineSb = XSbc;
    const Eigen::Vector3d& backprojLineEb = XEbc;
        
    // align each point of 3D map-line to 3D detected-line (by moving the camera frame and adjusting 3D map-line points)
    const Eigen::Vector3d S_Sb = mapLineS - backprojLineSb; // P-Bp
    const Eigen::Vector3d E_Eb = mapLineE - backprojLineEb; // Q-Bq
    const double sDeviation = S_Sb.norm(); // |P-Bp|
    const double eDeviation = E_Eb.norm(); // |Q-Bq|   
    return (sDeviation*sDeviation + eDeviation*eDeviation);
  }  

#if USE_ANALYTIC_JACS_FULL_STEREO 
  virtual void linearizeOplus();
#endif


  Eigen::Vector2d camProject(const Eigen::Vector3d & trans_xyz) const;
  
  Eigen::Vector3d camBackProject(const Eigen::Vector2d & p_uv, const double depth ) const;
  
  double fx, fy, cx, cy;
  Eigen::Vector3d XSbc; // backprojected start point w.r.t. camera  frame 
  Eigen::Vector3d XEbc; // backprojected end point w.r.t camera frame 
  double lineLenghtInv; // inverse of backprojected line length 
  double mu; // for weighting the point-point distance |P-Bp| (or |Q-Bq| ) w.r.t. distance point-line |(P-Bp) X (P-Bq)|/|Bq-Bp|  ( or |(Q-Bp) X (Q-Bq)|/|Bq-Bp| )
};


} // end namespace

