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
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified by Luigi Freda (2017-present)
 *  - added map lines and their covariances 
 *  - added map objects 
 *  - added parameters for optimization 
 *  - added compatibility with new g2o
*/


#include "Optimizer.h"


#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/core/block_solver.h"
#include "Thirdparty/g2o_new/install/include/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o_new/install/include/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o_new/install/include/g2o/solvers/eigen/linear_solver_eigen.h"
#include "Thirdparty/g2o_new/install/include/g2o/solvers/dense/linear_solver_dense.h"
#include "Thirdparty/g2o_new/install/include/g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#include "Thirdparty/g2o_new/install/include/g2o/types/sba/types_six_dof_expmap.h"
#include "Thirdparty/g2o_new/install/include/g2o/core/robust_kernel_impl.h"
#else
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#endif

#include "g2o/types_sba_line.h"
#include "g2o/types_six_dof_expmap2.h"
#include "g2o/types_seven_dof_expmap2.h"

#include "G2oTypes.h"
#include "G2oLineTypes.h"
#include "Converter.h"

#include<Eigen/StdVector>

#include "Map.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "MapObject.h"
#include "Converter.h"
#include "Utils.h"
#include "Geom2DUtils.h"

#include<mutex>

#define USE_LINES 1                                  // set to zero to completely exclude lines from optimization
#define USE_LINE_STEREO             (1 && USE_LINES)
#define USE_LINE_PRIOR_BA           (0 && USE_LINES) // <- not used, it was used just for testing!
#define USE_LINES_RIGHT_PROJECTION  (1 && USE_LINES)

#define USE_LINES_POSE_OPTIMIZATION (1 && USE_LINES)
#define USE_LINES_LOCAL_BA          (1 && USE_LINES)
#define USE_LINES_BA                (1 && USE_LINES) 
#define USE_LINES_EG                (1 && USE_LINES) 
#define USE_LINES_LOCAL_BA_INERTIAL (1 && USE_LINES)
#define USE_LINES_FULL_BA_INERTIAL  (1 && USE_LINES)
#define USE_LINES_MERGE_BA_INERTIAL (1 && USE_LINES) 
#define USE_LINES_STEREO_INERTIAL   (1 && USE_LINES)

#define USE_CAUCHY_KERNEL_FOR_LINES 0

#define USE_RGBD_POINT_REPROJ_ERR 0
#define USE_NEW_STEREO_POINT_INFORMATION_MAT 0

#define USE_NEW_LINE_INFORMATION_MAT 0  
#define USE_NEW_LINE_INFORMATION_MAT_STEREO ( 1 && USE_NEW_LINE_INFORMATION_MAT)

#define USE_OBJECTS 1                                   // set to zero to completely exclude objects from optimization
#define USE_OBJECTS_POSE_OPTIMIZATION (1 && USE_OBJECTS)
#define USE_OBJECTS_LOCAL_BA          (1 && USE_OBJECTS)
#define USE_OBJECTS_BA                (1 && USE_OBJECTS) 
#define USE_OBJECTS_EG                (1 && USE_OBJECTS) 
#define USE_OBJECTS_BA_INERTIAL       (0 && USE_OBJECTS)// WIP

#define USE_CAUCHY_KERNEL_FOR_OBJECTS 0 

#define USE_BA_VARIABLE_SIZE_SOLVER  (USE_LINES || USE_OBJECTS) // N.B.: this is compulsory when you use lines or objects 

#define VERBOSE 0
#define VERBOSE_POSE_OPTIMIZATION ( 1 && VERBOSE)
#define VERBOSE_LOCAL_BA          ( 1 && VERBOSE)
#define VERBOSE_BA                ( 1 && VERBOSE)
#define VERBOSE_EG                ( 1 && VERBOSE)

#define VERBOSE_3D_POINT_LINE_ALIGNMENT_ERROR 0
#define VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR 0
#define VERBOSE_OBJECTS_CHISQUARES  ( 1 && VERBOSE_BA)

#define CHECK_LINE_VALID_OBSERVATIONS 0

#define PRINT_COVARIANCE ( 0 && USE_G2O_NEW)

#include "OptimizableTypes.h"
#include "OptimizableLineTypes.h"


namespace PLVS2
{

const int Optimizer::kNumMinLineObservationsForBA = 1;  // we need at least 3 observations for decently condition mono line reprojection optimization 
const int Optimizer::kNumMinBAFailuresForErasingMapLine = 1;
const int Optimizer::kNumMinPosOptFailuresForLineGettingOutlier = 1;

const float Optimizer::kSigmaPointLinePrior = 0.1; // [m]   (used with USE_LINE_PRIOR_BA just for testing) < NOT USED 
const float Optimizer::kInvSigma2PointLinePrior = 1.0/(Optimizer::kSigmaPointLinePrior*Optimizer::kSigmaPointLinePrior);  //(used with USE_LINE_PRIOR_BA just for testing)

const float Optimizer::kMinLineSegmentLength = 0.05f; // [m]  /// < NOT USED to be removed  
const float Optimizer::kInvMinLineSegmentLength = 1.0/Optimizer::kMinLineSegmentLength; 

float Optimizer::skSigmaZFactor = 6; // 1, 3, 6, 9  (used for scaling the computed Utils::SigmaZ(depth) noise model)

const float Optimizer::kSigmaPointLineDistance = 0.05; // [m]  was 0.1 
const float Optimizer::kInvSigma2PointLineDistance = 1.0/(Optimizer::kSigmaPointLineDistance * Optimizer::kSigmaPointLineDistance); 

float Optimizer::skMuWeightForLine3dDist = 0.5; // weight default value 
float Optimizer::skSigmaLineError3D = Optimizer::kSigmaPointLineDistance;//*(1 + Optimizer::skMuWeightForLine3dDist);    
float Optimizer::skInvSigma2LineError3D = 1.0/(Optimizer::skSigmaLineError3D * Optimizer::skSigmaLineError3D ); 

inline float ComputeDepthSigma2(const float one_over_bf, const float depth)  
{
#if 0     
    const float sigma = Optimizer::skSigmaZFactor*(Utils::SigmaZ(depth)); // depth sigma from "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
#else  
    // BAD-SLAM model
    const float sigma = Optimizer::skSigmaZFactor*(0.1*one_over_bf*depth*depth); // depth sigma from "BAD-SLAM" paper for depth sensor 
#endif 
    return sigma*sigma;    
}

void SetStereoPointInformationMat(Eigen::Matrix3d& Info, const float invSigma2, const float bf, const float one_over_bf, const float depth)
{
    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the error are weighted uniformly according to the detection uncertainty)
    
    /// <  N.B.: we assume the matrix Info has been initialized to zero
    const float depth2 = depth*depth; 
    const float depth4 = depth2*depth2; 
    const float sigmaz2 = ComputeDepthSigma2(one_over_bf, depth)/invSigma2; // depth variance
    const float bf2sigmaz2_over_d4 = bf*bf*sigmaz2/depth4;
    const float one_over_bf2sigmaz2_over_d4 = 1./bf2sigmaz2_over_d4;
    
    //[ (bf2sigmaz2_over_d4 + sigmap2)/(bf2sigmaz2_over_d4*sigmap2),         0, -1/bf2sigmaz2_over_d4]
    //[                                                           0, 1/sigmap2,                     0]
    //[                                       -1/bf2sigmaz2_over_d4,         0,  1/bf2sigmaz2_over_d4]    
    Info(0,0) = one_over_bf2sigmaz2_over_d4 + invSigma2;
    Info(0,2) = Info(2,0) = -one_over_bf2sigmaz2_over_d4;
    Info(1,1) = invSigma2;
    Info(2,2) = one_over_bf2sigmaz2_over_d4; 
}

void SetRgbdPointInformationMat(Eigen::Matrix3d& Info, const float invSigma2, const float one_over_bf, const float& depth)
{
    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the error are weighted uniformly according to the detection uncertainty)
    
    /// <  N.B.: we assume the matrix Info has been initialized to zero
    const float detphSigmaz2 = ComputeDepthSigma2(one_over_bf, depth);  
    Info(0,0) = invSigma2;
    Info(1,1) = invSigma2;
    Info(2,2) = invSigma2/detphSigmaz2;
}

void Set2DLineInformationMat(double& invSigma2_d2Dp, double& invSigma2_d2Dq, const float& sigmali2, 
                           const float& px, const float& py, 
                           const float& qx, const float& qy, 
                           const float& nx, const float& ny, 
                           const Eigen::Vector2d& projMapP, const Eigen::Vector2d& projMapQ)
{
     /// <  N.B.: we assume the matrix Info has been initialized to zero
    
    Eigen::Vector2d l;
    l << (qy-py),(px-qx);
    const float nx_ny = nx*ny;
    const float nx2 = nx*nx;
    const float ny2 = ny*ny;
    const float nx2_1 = (nx2 - 1.f); 
    const float ny2_1 = (ny2 - 1.f);
    const float two_sigmali2 = 2.f * sigmali2;
    const float two_sigmali2_nx2_ny2 = two_sigmali2*nx2*ny2;
    const float two_sigmali2_nx_ny = two_sigmali2*nx_ny;
    const float sym_term = two_sigmali2_nx_ny*(nx2_1)  + two_sigmali2_nx_ny*(ny2_1);
    
    
    Eigen::Matrix2d Sigma_n_x_l2;
    //Sigma_n_x_l2    2*sigmali2*(nx^2 - 1)^2 + 2*nx^2*ny^2*sigmali2,                       
    //                2*nx*ny*sigmali2*(nx^2 - 1) + 2*nx*ny*sigmali2*(ny^2 - 1)
    //                2*nx*ny*sigmali2*(nx^2 - 1) + 2*nx*ny*sigmali2*(ny^2 - 1),            
    //                2*sigmali2*(ny^2 - 1)^2 + 2*nx^2*ny^2*sigmali2;
    Sigma_n_x_l2 << two_sigmali2*(nx2_1*nx2_1) + two_sigmali2_nx2_ny2,        
                    sym_term,
                    sym_term,   
                    two_sigmali2*(ny2_1*ny2_1) + two_sigmali2_nx2_ny2;

    const float nx_ny_px = nx_ny*px;
    const float nx_ny_py = nx_ny*py;
    const float px_nx2_1 = px*(nx2_1);
    const float py_ny2_1 = py*(ny2_1);
    // sigma_h_x_l2 = sigmali2*(px*(nx^2 - 1) + nx*ny*py)^2 
    //              + sigmali2*(py*(ny^2 - 1) + nx*ny*px)^2 
    //              + sigmali2*(px - qx + px*(nx^2 - 1) + nx*ny*py)^2 
    //              + sigmali2*(py - qy + py*(ny^2 - 1) + nx*ny*px)^2
    float sigma_h_x_l2 = sigmali2*( Utils::Pow2(px_nx2_1 + nx_ny_py) + 
                                    Utils::Pow2(py_ny2_1 + nx_ny_px) + 
                                    Utils::Pow2( l[1] + px_nx2_1 + nx_ny_py) + 
                                    Utils::Pow2(-l[0] + py_ny2_1 + nx_ny_px) );
    
    const float lnorm = l.norm();
    const float lnorm2 = lnorm*lnorm;
    
    const float sigma2_d2Dp = (projMapP.dot(Sigma_n_x_l2*projMapP) + sigma_h_x_l2)/lnorm2;
    const float sigma2_d2Dq = (projMapQ.dot(Sigma_n_x_l2*projMapQ) + sigma_h_x_l2)/lnorm2;
    
    //std::cout << "sigma2_d2Dp: " << sigma2_d2Dp << ", sigma2_d2Dq: " << sigma2_d2Dq << std::endl;
    invSigma2_d2Dp = 1./sigma2_d2Dp;
    invSigma2_d2Dq = 1./sigma2_d2Dq;
}
        
void SetSigmaBetaMats(Eigen::Matrix3d& BetaMatP, Eigen::Matrix3d& BetaMatQ,
                const float& sigmali2, const float& sigmaz2P, const float& sigmaz2Q,  
                const float& fx, const float& fy,
                const Eigen::Vector2d& projMapP, const float& depthP,
                const Eigen::Vector2d& projMapQ, const float& depthQ)
{
    /// <  N.B.: we assume the matrix Info has been initialized to zero
//[ sigmaz2*xc^2 + (d^2*sigmali2)/fx^2,                      sigmaz2*xc*yc, sigmaz2*xc]
//[                      sigmaz2*xc*yc, sigmaz2*yc^2 + (d^2*sigmali2)/fy^2, sigmaz2*yc]
//[                         sigmaz2*xc,                         sigmaz2*yc,    sigmaz2]    
    
    const float fx2 = fx*fx;
    const float fy2 = fy*fy; 
    
    const float& xP = projMapP[0];
    const float& yP = projMapP[1];
    const float d2_P = depthP*depthP; 
    const float d2_sigmali2_P = d2_P*sigmali2;
    BetaMatP(0,0) = sigmaz2P*xP*xP + d2_sigmali2_P/fx2;  // sigmaz2*xc^2 + (d^2*sigmali2)/fx^2
    BetaMatP(1,1) = sigmaz2P*yP*yP + d2_sigmali2_P/fy2;  // sigmaz2*yc^2 + (d^2*sigmali2)/fy^2
    BetaMatP(2,2) = sigmaz2P;
            
    const float sigmaz2P_xP = sigmaz2P*xP;
    BetaMatP(0,1) = BetaMatP(1,0) = sigmaz2P_xP*yP; // sigmaz2*xc*yc
    BetaMatP(0,2) = BetaMatP(2,0) = sigmaz2P_xP;    // sigmaz2*xc
    
    BetaMatP(1,2) = BetaMatP(2,1) =  sigmaz2P*yP; // sigmaz2*yc
    
    // -- // 
    
    const float& xQ = projMapQ[0];
    const float& yQ = projMapQ[1];
    const float d2_Q = depthQ*depthQ; 
    const float d2_sigmali2_Q = d2_Q*sigmali2;
    BetaMatQ(0,0) = sigmaz2Q*xQ*xQ + d2_sigmali2_Q/fx2;  // sigmaz2*xc^2 + (d^2*sigmali2)/fx^2
    BetaMatQ(1,1) = sigmaz2Q*yQ*yQ + d2_sigmali2_Q/fy2;  // sigmaz2*yc^2 + (d^2*sigmali2)/fy^2
    BetaMatQ(2,2) = sigmaz2Q;
            
    const float sigmaz2Q_xQ = sigmaz2Q*xQ;
    BetaMatQ(0,1) = BetaMatQ(1,0) = sigmaz2Q_xQ*yQ; // sigmaz2*xc*yc
    BetaMatQ(0,2) = BetaMatQ(2,0) = sigmaz2Q_xQ;    // sigmaz2*xc
    
    BetaMatQ(1,2) = BetaMatQ(2,1) =  sigmaz2Q*yQ; // sigmaz2*yc    
}        

void Set3DLineInformationMat(double& invSigma2_d3Dp, double& invSigma2_d3Dq, 
                const float& sigmali2, const int& octave,
                const float& fx, const float& fy, const float one_over_bf,
                const Eigen::Vector2d& projMapP, 
                const Eigen::Vector2d& projMapQ,
                const Eigen::Vector3d& mapP, const Eigen::Vector3d& mapQ,
                const Eigen::Vector3d& backprojP, const Eigen::Vector3d& backprojQ)
{
    const float& depthP = backprojP[2];
    const float& depthQ = backprojQ[2];
    
    const float sigmaz2P = ComputeDepthSigma2(one_over_bf, depthP) * sigmali2; //Utils::Pow2(actualOctave*Optimizer::skSigmaZFactor*Utils::SigmaZ(depthP)); // depth variance in P
    const float sigmaz2Q = ComputeDepthSigma2(one_over_bf, depthQ) * sigmali2; //Utils::Pow2(actualOctave*Optimizer::skSigmaZFactor*Utils::SigmaZ(depthQ)); // depth variance in Q
 
    Eigen::Matrix3d SigmaBetaMatP, SigmaBetaMatQ;
    SetSigmaBetaMats(SigmaBetaMatP, SigmaBetaMatQ,
                sigmali2, sigmaz2P, sigmaz2Q,  
                fx, fy,
                projMapP, depthP,
                projMapQ, depthQ); 

    const Eigen::Vector3d& Pc = mapP;
    const Eigen::Vector3d& Qc = mapQ;

    const Eigen::Vector3d& Bpc = backprojP;
    const Eigen::Vector3d& Bqc = backprojQ;

    const Eigen::Vector3d DeltaB = Bpc-Bqc;
    const double& DeltaB_x = DeltaB[0];
    const double& DeltaB_y = DeltaB[1];
    const double& DeltaB_z = DeltaB[2];
    const double normDeltaB = DeltaB.norm(); 
    const double normDeltaB3 = normDeltaB*normDeltaB*normDeltaB;
    const double inv_normDeltaB3 = 1./normDeltaB3;

    const Eigen::Vector3d P_Bp = (Pc - Bpc);
    const double& P_Bp_x = P_Bp[0];
    const double& P_Bp_y = P_Bp[1];
    const double& P_Bp_z = P_Bp[2];
    
    const Eigen::Vector3d P_Bq = (Pc - Bqc);
    const double& P_Bq_x = P_Bq[0];
    const double& P_Bq_y = P_Bq[1];
    const double& P_Bq_z = P_Bq[2];    

    const Eigen::Vector3d Q_Bp = (Qc - Bpc);
    const double& Q_Bp_x = Q_Bp[0];
    const double& Q_Bp_y = Q_Bp[1];
    const double& Q_Bp_z = Q_Bp[2];  
    
    const Eigen::Vector3d Q_Bq = (Qc - Bqc);
    const double& Q_Bq_x = Q_Bq[0];
    const double& Q_Bq_y = Q_Bq[1];
    const double& Q_Bq_z = Q_Bq[2];  

    const Eigen::Vector3d VP = P_Bp.cross(P_Bq); // ((P-Bp) X (P-Bq))
    const double& VPx = VP[0];
    const double& VPy = VP[1];
    const double& VPz = VP[2];
    const double normVP = VP.norm();

    const Eigen::Vector3d VQ = Q_Bp.cross(Q_Bq); // ((Q-Bp) X (Q-Bq))
    const double& VQx = VQ[0];
    const double& VQy = VQ[1];
    const double& VQz = VQ[2]; 
    const double normVQ = VQ.norm();
    
    //d3D_dBp_inP = VP'*deltaQ_skew_inP/(normDeltaB*normVP) -normVP*DeltaB'/normDeltaB3
    //where deltaQ_inP  = [P_Bq_x;P_Bq_y;P_Bq_z]; 
    //d3D_dBp_inP = [ (P_Bq_z*VPy - P_Bq_y*VPz)/(normVP*normDeltaB) - (DeltaB_x*normVP)/normDeltaB3, 
    //              - (P_Bq_z*VPx - P_Bq_x*VPz)/(normVP*normDeltaB) - (DeltaB_y*normVP)/normDeltaB3, 
    //                (P_Bq_y*VPx - P_Bq_x*VPy)/(normVP*normDeltaB) - (DeltaB_z*normVP)/normDeltaB3]
    Eigen::Vector3d d3D_dBp_inP;
    const double inv_normVP_normDeltaB = 1./(normVP*normDeltaB);
    const double normVP_inv_normDeltaB3 = normVP*inv_normDeltaB3;
    d3D_dBp_inP  <<  (P_Bq_z*VPy - P_Bq_y*VPz)*inv_normVP_normDeltaB - DeltaB_x*normVP_inv_normDeltaB3, 
                   - (P_Bq_z*VPx - P_Bq_x*VPz)*inv_normVP_normDeltaB - DeltaB_y*normVP_inv_normDeltaB3, 
                     (P_Bq_y*VPx - P_Bq_x*VPy)*inv_normVP_normDeltaB - DeltaB_z*normVP_inv_normDeltaB3;
 
    //d3D_dBp_inQ = VQ'*deltaQ_skew_inQ/(normDeltaB*normVQ) -normVQ*DeltaB'/normDeltaB3
    //where deltaQ_inQ = [Q_Bq_x;Q_Bq_y;Q_Bq_z];
    //d3D_dBp_inQ = [ (Q_Bq_z*VQy - Q_Bq_y*VQz)/(normVQ*normDeltaB) - (DeltaB_x*normVQ)/normDeltaB3, 
    //              - (Q_Bq_z*VQx - Q_Bq_x*VQz)/(normVQ*normDeltaB) - (DeltaB_y*normVQ)/normDeltaB3, 
    //                (Q_Bq_y*VQx - Q_Bq_x*VQy)/(normVQ*normDeltaB) - (DeltaB_z*normVQ)/normDeltaB3]
    Eigen::Vector3d d3D_dBp_inQ;  
    const double inv_normVQ_normDeltaB = 1./(normVQ*normDeltaB);    
    const double normVQ_inv_normDeltaB3 = normVQ*inv_normDeltaB3;      
    d3D_dBp_inQ  << (Q_Bq_z*VQy - Q_Bq_y*VQz)*inv_normVQ_normDeltaB - DeltaB_x*normVQ_inv_normDeltaB3, 
                  - (Q_Bq_z*VQx - Q_Bq_x*VQz)*inv_normVQ_normDeltaB - DeltaB_y*normVQ_inv_normDeltaB3, 
                    (Q_Bq_y*VQx - Q_Bq_x*VQy)*inv_normVQ_normDeltaB - DeltaB_z*normVQ_inv_normDeltaB3;

    //d3D_dBq_inP = -VP'*deltaP_skew_inP/(normDeltaB*normVP) +normVP*DeltaB'/normDeltaB3
    //where deltaP_inP = [P_Bp_x;P_Bp_y;P_Bp_z];
    //d3D_dBq_inP = [ (DeltaB_x*normVP)/normDeltaB3 - (P_Bp_z*VPy - P_Bp_y*VPz)/(normVP*normDeltaB), 
    //                (P_Bp_z*VPx - P_Bp_x*VPz)/(normVP*normDeltaB) + (DeltaB_y*normVP)/normDeltaB3, 
    //                (DeltaB_z*normVP)/normDeltaB3 - (P_Bp_y*VPx - P_Bp_x*VPy)/(normVP*normDeltaB)]
    Eigen::Vector3d d3D_dBq_inP;
    d3D_dBq_inP <<  DeltaB_x*normVP_inv_normDeltaB3 - (P_Bp_z*VPy - P_Bp_y*VPz)*inv_normVP_normDeltaB, 
                    (P_Bp_z*VPx - P_Bp_x*VPz)*inv_normVP_normDeltaB + (DeltaB_y*normVP)*inv_normDeltaB3, 
                    DeltaB_z*normVP_inv_normDeltaB3 - (P_Bp_y*VPx - P_Bp_x*VPy)*inv_normVP_normDeltaB;    

    //d3D_dBq_inQ = -VQ'*deltaP_skew_inQ/(normDeltaB*normVQ) +normVQ*DeltaB'/normDeltaB3
    //where deltaP_inQ = [Q_Bp_x;Q_Bp_y;Q_Bp_z];
    //d3D_dBq_inQ = [ (DeltaB_x*normVQ)/normDeltaB3 - (Q_Bp_z*VQy - Q_Bp_y*VQz)/(normVQ*normDeltaB), 
    //                (Q_Bp_z*VQx - Q_Bp_x*VQz)/(normVQ*normDeltaB) + (DeltaB_y*normVQ)/normDeltaB3, 
    //                (DeltaB_z*normVQ)/normDeltaB3 - (Q_Bp_y*VQx - Q_Bp_x*VQy)/(normVQ*normDeltaB)]
    Eigen::Vector3d d3D_dBq_inQ;
    d3D_dBq_inQ <<  DeltaB_x*normVQ_inv_normDeltaB3 - (Q_Bp_z*VQy - Q_Bp_y*VQz)*inv_normVQ_normDeltaB, 
                    (Q_Bp_z*VQx - Q_Bp_x*VQz)*inv_normVQ_normDeltaB + (DeltaB_y*normVQ)*inv_normDeltaB3, 
                    DeltaB_z*normVQ_inv_normDeltaB3 - (Q_Bp_y*VQx - Q_Bp_x*VQy)*inv_normVQ_normDeltaB;   
 
    // ddP_dBp_inP = deltaP_inP/normDeltaP
    const Eigen::Vector3d ddP_dBp_inP = P_Bp.normalized();
    
    //ddP_dBq_inQ = deltaQ_inQ/normDeltaQ
    const Eigen::Vector3d ddP_dBq_inQ = Q_Bq.normalized();
    
    const Eigen::Vector3d d3D_dBp_plus_ddP_dBp_inP = d3D_dBp_inP + Optimizer::skMuWeightForLine3dDist * ddP_dBp_inP;
    const Eigen::Vector3d d3D_dBq_plus_ddP_dBq_inQ = d3D_dBq_inQ + Optimizer::skMuWeightForLine3dDist * ddP_dBq_inQ;
    
    const double sigma2_d3Dp= d3D_dBp_plus_ddP_dBp_inP.dot(SigmaBetaMatP*d3D_dBp_plus_ddP_dBp_inP) + d3D_dBq_inP.dot(SigmaBetaMatQ*d3D_dBq_inP);
    
    const double sigma2_d3Dq = d3D_dBp_inQ.dot(SigmaBetaMatP*d3D_dBp_inQ) + d3D_dBq_plus_ddP_dBq_inQ.dot(SigmaBetaMatQ*d3D_dBq_plus_ddP_dBq_inQ); 
    
    //std::cout << "sigma2_d3Dp: " << sigma2_d3Dp << ", sigma2_d3Dq: " << sigma2_d3Dq << std::endl;
        
    invSigma2_d3Dp = 1.0/sigma2_d3Dp;
  
    invSigma2_d3Dq = 1.0/sigma2_d3Dq;    
}


bool sortByVal(const pair<MapPointPtr , int> &a, const pair<MapPointPtr , int> &b)
{
    return (a.second < b.second);
}

bool sortByValLines(const pair<MapLinePtr , int> &a, const pair<MapLinePtr , int> &b)
{
    return (a.second < b.second);
}

/// < < < < < <  < < < < <  < < < < <  < < < < <  < < < < <  < < < < < 

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPointPtr> vpMPoints = pMap->GetAllMapPoints();
    
#if USE_LINES_BA
    vector<MapLinePtr> vpMLines  = pMap->GetAllMapLines();
#else
    vector<MapLinePtr> vpMLines;
#endif
    
#if USE_OBJECTS_BA
    vector<MapObjectPtr> vpMObjects  = pMap->GetAllMapObjects();
#else
    vector<MapObjectPtr> vpMObjects;
#endif    
    
    BundleAdjustment(vpKFs, vpMPoints, vpMLines, vpMObjects, nIterations, pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<KeyFramePtr> &vpKFs, const vector<MapPointPtr> &vpMPoints, 
                                 const vector<MapLinePtr> &vpMLines, const vector<MapObjectPtr > &vpMObjects,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
#if VERBOSE_BA
    std::cout << "******************************" << std::endl; 
    std::cout << "Optimizer::BundleAdjustment() " << std::endl; 
    std::cout << "******************************" << std::endl; 
#endif
            
    vector<bool> vbNotIncludedMPoints;
    vbNotIncludedMPoints.resize(vpMPoints.size());
    
#if USE_LINES_BA    
    vector<bool> vbNotIncludedMLines;
    vbNotIncludedMLines.resize(vpMLines.size());
#endif
    
#if USE_OBJECTS_BA    
    vector<bool> vbNotIncludedMObjects;
    vbNotIncludedMObjects.resize(vpMObjects.size());
#endif    

    Map* pMap = vpKFs[0]->GetMap();

    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver;

    
#if USE_LINES_BA || USE_OBJECTS_BA
    const size_t numLines = vpMLines.size();
    const size_t numObjects = vpMObjects.size();    
    
#if USE_BA_VARIABLE_SIZE_SOLVER    
    if( (numLines>0) || (numObjects>0) )
    {
        
#ifdef USE_G2O_NEW        
        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
        g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);        
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW
        
    }
    else
#endif // USE_BA_VARIABLE_SIZE_SOLVER       
    {
        
#ifdef USE_G2O_NEW        
        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>()));        
#else        
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);        
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW   
        
    }
    
#else    // USE_LINES_BA || USE_OBJECTS_BA
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);    
    
#endif  // USE_LINES_BA || USE_OBJECTS_BA
    
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    long unsigned int maxPointId = 0; 
    long unsigned int maxLineId = 0;

    const int nExpectedSize = (vpKFs.size())*vpMPoints.size();

    vector<PLVS2::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<PLVS2::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

#if USE_LINES_BA
    const int nLinesExpectedSize = (vpKFs.size())*vpMLines.size();
        
    vector<g2o::EdgeSE3ProjectLine*> vpLineEdgesMono;
    vpLineEdgesMono.reserve(nLinesExpectedSize);

    vector<PLVS2::EdgeSE3ProjectLineToBody*> vpLineEdgesBody;
    vpLineEdgesBody.reserve(nLinesExpectedSize);

    vector<KeyFramePtr> vpLineEdgeKFMono;
    vpLineEdgeKFMono.reserve(nLinesExpectedSize);

    vector<KeyFramePtr> vpLineEdgeKFBody;
    vpLineEdgeKFBody.reserve(nLinesExpectedSize);

    vector<MapLinePtr> vpMapLineEdgeMono;
    vpMapLineEdgeMono.reserve(nLinesExpectedSize);

    vector<MapLinePtr> vpMapLineEdgeBody;
    vpMapLineEdgeBody.reserve(nLinesExpectedSize);

    vector<g2o::EdgeSE3ProjectStereoLine*> vpLineEdgesStereo;
    vpLineEdgesStereo.reserve(nLinesExpectedSize);

    vector<KeyFramePtr> vpLineEdgeKFStereo;
    vpLineEdgeKFStereo.reserve(nLinesExpectedSize);

    vector<MapLinePtr> vpMapLineEdgeStereo;
    vpMapLineEdgeStereo.reserve(nLinesExpectedSize);
#endif 
    
#if USE_OBJECTS_BA     
    const int nObjectExpectedSize = vpMObjects.size()*5; // 5 views per object on the average
    vector<g2o::EdgeSim3SE3*> vpObjectEdges;
    vpObjectEdges.reserve(nObjectExpectedSize);

    vector<KeyFramePtr> vpObjectEdgeKF;
    vpObjectEdgeKF.reserve(nObjectExpectedSize);

    vector<MapObjectPtr> vpMapObjectEdge;
    vpMapObjectEdge.reserve(nObjectExpectedSize);
#endif 
    
    
    // Set KeyFrame vertices

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKF->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==pMap->GetInitKFid());
        vSE3->setFixed(pKF->mbFixed);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);  // chi-squared 2 DOFs
    const float thHuber3D = sqrt(7.815); // chi-squared 3 DOFs
    
    const float thHuberLineMono = sqrt(5.99);    // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49);  // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    
    const float thHuberObjectTimesSigma = sqrt(3);  // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    // Set MapPoint vertices
    for(size_t i=0; i<vpMPoints.size(); i++)
    {
        MapPointPtr pMP = vpMPoints[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        const int id = pMP->mnId+maxKFid+1;
        //if(id>maxPointId) maxPointId = id; 
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        
        g2o::OptimizableGraph::Vertex* vertexPoint = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        if(vertexPoint == NULL)
            continue;
        
       const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));
            //if(optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
            //    continue;
            if(vertexKF == NULL)
                    continue;

            nEdges++;

            const int leftIndex = get<0>(mit->second);

            if(leftIndex != -1 && pKF->mvuRight[leftIndex]<0)
            {
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                PLVS2::EdgeSE3ProjectXYZ* e = new PLVS2::EdgeSE3ProjectXYZ();

                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexPoint);
                e->setVertex(1, vertexKF);                
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMP);
            }
            else if(leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) //Stereo observation
            {
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double,3,1> obs;

#if !USE_RGBD_POINT_REPROJ_ERR                    
                const float kp_ur = pKF->mvuRight[leftIndex];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
#else
                const float kpDelta = pKF->mvDepth[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kpDelta;

                g2o::EdgeRgbdSE3ProjectXYZ* e = new g2o::EdgeRgbdSE3ProjectXYZ();                    
#endif

                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexPoint);
                e->setVertex(1, vertexKF);                     
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
     
#if !USE_RGBD_POINT_REPROJ_ERR                 
    #if !USE_NEW_STEREO_POINT_INFORMATION_MAT                
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
    #else
                Eigen::Matrix3d Info = Eigen::Matrix3d::Zero(); 
                SetStereoPointInformationMat(Info, invSigma2, pKF->mbf, pKF->mbfInv, pKF->mvDepth[mit->second]);
    #endif          
#else
                Eigen::Matrix3d Info = Eigen::Matrix3d::Zero(); 
                SetRgbdPointInformationMat(Info, invSigma2, pKF->mbfInv, pKF->mvDepth[mit->second]);
#endif
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
#if !USE_RGBD_POINT_REPROJ_ERR                  
                e->bf = pKF->mbf;
#endif
                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMP);
            }

            if(pKF->mpCamera2)
            {
                int rightIndex = get<1>(mit->second);

                if(rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()){
                    rightIndex -= pKF->NLeft;

                    Eigen::Matrix<double,2,1> obs;
                    cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                    obs << kp.pt.x, kp.pt.y;

                    PLVS2::EdgeSE3ProjectXYZToBody *e = new PLVS2::EdgeSE3ProjectXYZToBody();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    const Sophus::SE3f Trl = pKF-> GetRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                    e->pCamera = pKF->mpCamera2;

                    optimizer.addEdge(e);
                    vpEdgesBody.push_back(e);
                    vpEdgeKFBody.push_back(pKF);
                    vpMapPointEdgeBody.push_back(pMP);
                }
            }
        }


        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMPoints[i]=true;
        }
        else
        {
            vbNotIncludedMPoints[i]=false;
        }
    }

    maxPointId = maxKFid+1+MapPoint::GetCurrentMaxId(); 
    
#if USE_LINES_BA    // ---------------------------------------------------------
    // Set MapLine vertices
    for(size_t i=0; i<vpMLines.size(); i++)
    {
        const MapLinePtr pML = vpMLines[i];
        if(pML->isBad())
            continue;
        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);                          
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());
        const int id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine);

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();
        //if(observations.size() < kNumMinLineObservationsForBA) continue;        

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;
            
            g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));                        
            if(vertexKF == NULL)
                continue;              

            nEdges++;
            
            const int leftIndex = get<0>(mit->second);
            float uRightLineStart = -1;
            float uRightLineEnd = -1;
            if(leftIndex != -1 && !pKF->mvuRightLineStart.empty()) 
            {
                uRightLineStart = pKF->mvuRightLineStart[leftIndex];
                uRightLineEnd = pKF->mvuRightLineEnd[leftIndex];
            }
                            
            // Monocular observation
#if USE_LINE_STEREO                
            if( (leftIndex != -1) && ( uRightLineStart<0 || uRightLineEnd<0 ) )
#endif            
            {
                const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesUn[leftIndex];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
            
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                g2o::EdgeSE3ProjectLine* e = new g2o::EdgeSE3ProjectLine();
                    
                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexLine);
                e->setVertex(1, vertexKF);  
                e->setMeasurement(obs);
                
                //e->pCamera = pKF->mpCamera; // this is enabled with the new types in OptimizableLineTypes.h                
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;                
                
#if !USE_NEW_LINE_INFORMATION_MAT   
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
#else
                const float sigma2 = pKF->mvLineLevelSigma2[klUn.octave];

                Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                Eigen::Vector2d projMapP, projMapQ;
                e->getMapLineProjections(projMapP, projMapQ);
                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
                e->setInformation(Info);
#endif                 

                if(bRobust)
                {
        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(thHuberLineMono);
        #endif    
                    e->setRobustKernel(rk);
                }

                optimizer.addEdge(e);
                
                vpLineEdgesMono.push_back(e);
                vpLineEdgeKFMono.push_back(pKF);
                vpMapLineEdgeMono.push_back(pML);                
            }
#if USE_LINE_STEREO              
            else if ( (leftIndex != -1) && ( uRightLineStart>=0 && uRightLineEnd>=0 ) ) //Stereo observation
            {
                const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesUn[leftIndex];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);   

                g2o::EdgeSE3ProjectStereoLine* e = new g2o::EdgeSE3ProjectStereoLine();

                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexLine);
                e->setVertex(1, vertexKF); 
                e->setMeasurement(obs);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;  
                
                // the following two are actually derived observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pKF->mvDepthLineStart[leftIndex]);
                e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pKF->mvDepthLineEnd[leftIndex]);
                        
                e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 
                e->mu = Optimizer::skMuWeightForLine3dDist;
                
                e->init(); // here we check the match between Bp and P (Bq and Q)
                
    #if !USE_NEW_LINE_INFORMATION_MAT                   
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                Info(0,0)*=invSigma2;
                Info(1,1)*=invSigma2;
                Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
    #else
                const float sigma2 = pKF->mvLineLevelSigma2[klUn.octave];
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                Eigen::Vector2d projMapP, projMapQ;
                Eigen::Vector3d mapP, mapQ;
                e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                Eigen::Vector3d &backprojP = e->XSbc;
                Eigen::Vector3d &backprojQ = e->XEbc; 

                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
        #if USE_NEW_LINE_INFORMATION_MAT_STEREO                
                Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                sigma2, klUn.octave,
                                pKF->fx, pKF->fy, pKF->mbfInv, 
                                projMapP, projMapQ, 
                                mapP, mapQ,
                                backprojP, backprojQ);    
        #else
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
        #endif
                
    #endif
                
                e->setInformation(Info);

                if(bRobust)
                {
    #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
    #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(thHuberLineStereo);
    #endif    
                    e->setRobustKernel(rk);
                }

                optimizer.addEdge(e);
                
                vpLineEdgesStereo.push_back(e);
                vpLineEdgeKFStereo.push_back(pKF);
                vpMapLineEdgeStereo.push_back(pML);                
            }
            
            if(pKF->mpCamera2)
            {
                int rightIndex = get<1>(mit->second);

                if(rightIndex != -1 && rightIndex < pKF->mvKeyLinesRightUn.size())
                {
                    rightIndex -= pKF->NlinesLeft;

                    const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesRightUn[rightIndex];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
                    Eigen::Matrix<double,3,1> obs;
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                    PLVS2::EdgeSE3ProjectLineToBody* e = new PLVS2::EdgeSE3ProjectLineToBody();
                        
                    //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                    e->setVertex(0, vertexLine);
                    e->setVertex(1, vertexKF);  
                    e->setMeasurement(obs);
                    
                    const Sophus::SE3f Trl = pKF-> GetRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                    e->pCamera = pKF->mpCamera2;                       
                    
                    const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);        

                    if(bRobust)
                    {
            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineMono);
            #endif    
                        e->setRobustKernel(rk);
                    }

                    optimizer.addEdge(e);
                    
                    vpLineEdgesBody.push_back(e);
                    vpLineEdgeKFBody.push_back(pKF);
                    vpMapLineEdgeBody.push_back(pML);  
                }
            }

#endif   // #if USE_LINE_STEREO        
        }

        //if(nEdges<kNumMinLineObservationsForBA)  <- in order to use this one should also remove the edges! 
        if(nEdges==0)
        {
            optimizer.removeVertex(vLine);
            vbNotIncludedMLines[i]=true;
        }
        else
        {
            vbNotIncludedMLines[i]=false;
        }
        
    } // end for(size_t i=0; i<vpMLines.size(); i++)
#endif
    
    
    maxLineId = maxPointId+1+MapLine::GetCurrentMaxId();    
    
#if USE_OBJECTS_BA    // ---------------------------------------------------------
    
    bool bFixScale = false;    
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    const int nExpectedSizeObjects = vpMObjects.size()*5; // 5 views per object on the average

    vector<g2o::EdgeSim3SE3*> vpEdgesObject;
    vpEdgesObject.reserve(nExpectedSizeObjects);
    
    std::vector<double> vEdgesObjectSquaredErrors;
    vEdgesObjectSquaredErrors.reserve(nExpectedSizeObjects);
    
    // Set MapObject vertices
    for(size_t i=0; i<vpMObjects.size(); i++)
    {
        MapObjectPtr pMObj = vpMObjects[i];
        if(pMObj->isBad())
            continue;
        g2o::VertexSim3Expmap* vObject = new g2o::VertexSim3Expmap();
        Eigen::Matrix<double,3,3> Row = pMObj->GetRotation().cast<double>();
        Eigen::Matrix<double,3,1> tow = pMObj->GetTranslation().cast<double>();
        double objectScale = pMObj->GetScale();
        g2o::Sim3 Sow(Row,tow,1./objectScale); // Sow = [Row/s, tow; 0, 1]  
        vObject->setEstimate(Sow);
        int id = pMObj->mnId+maxLineId+1;
        vObject->setId(id);
        vObject->setMarginalized(true);
        vObject->_fix_scale = bFixScale;        
        optimizer.addVertex(vObject);

        g2o::OptimizableGraph::Vertex* vertexObject = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,ObjectObservation> observations = pMObj->GetObservations();   

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,ObjectObservation>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;
            
            g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));                        
            if(vertexKF == NULL)
                continue;              

            nEdges++;
            
            const ObjectObservation& observation = mit->second;

            //if(!observation.bFromKF) continue; 

            Sophus::SE3f Tko = observation.GetSE3();
            Eigen::Matrix<double,3,3> Rko = Tko.rotationMatrix().cast<double>();
            Eigen::Matrix<double,3,1> tko = Tko.translation().cast<double>();                
            double observedScale = observation.fScale;
                
            const g2o::Sim3 Sko(Rko,tko,observedScale); // Sko = [s*Rko, tko; 0, 1]             

            g2o::EdgeSim3SE3* e = new g2o::EdgeSim3SE3();
            e->setVertex(0, vertexObject);  // Sim3 Sow              
            e->setVertex(1, vertexKF); // SE3 Tkw                   
            e->setMeasurement(Sko);
            e->setInformation(matLambda);

            if(bRobust)
            {
        #if USE_CAUCHY_KERNEL_FOR_OBJECTS
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberObjectTimesSigma);
        #endif                 

                e->setRobustKernel(rk);
            }
            
            vpEdgesObject.push_back(e);
            
            e->computeError();
            vEdgesObjectSquaredErrors.push_back(e->chi2()); // here we accumulate the squared errors (given that matLambda is the identity)
            //std::cout << "chi2: " << vEdgesObjectSquaredErrors[vEdgesObjectSquaredErrors.size()-1] << std::endl;            

            optimizer.addEdge(e);
   
            vpObjectEdges.push_back(e);
            vpObjectEdgeKF.push_back(pKF);
            vpMapObjectEdge.push_back(pMObj);
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vObject);
            vbNotIncludedMObjects[i]=true;
        }
        else
        {
            vbNotIncludedMObjects[i]=false;
        }
        
    } // end for(size_t i=0; i<vpMObjects.size(); i++)
    
    
    // we need this step in order to set a decent information matrix 
    if(vEdgesObjectSquaredErrors.size()>0)
    {
        double sigmaEdgesObjectsSquared = Utils::FindSigmaSquared(vEdgesObjectSquaredErrors);   
        double invSigmaEdgesObjectsSquared = 1./sigmaEdgesObjectsSquared;
        for(size_t i=0, iend=vEdgesObjectSquaredErrors.size(); i<iend;i++)
        {
            vpEdgesObject[i]->information() *= invSigmaEdgesObjectsSquared;  // N.B.: in this way one has e->chi2() = err_k^2/sigma_err^2
        }
    }    
    
#endif   // USE_OBJECTS_BA
    
//------------------------------------------------------------------------------

    // Optimize!
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        if(!vSE3)
            continue; 
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
        }
        else
        {
            /*if(!vSE3->fixed())
            {
                //cout << "KF " << pKF->mnId << ": " << endl;
                pKF->mHessianPose = cv::Mat(6, 6, CV_64F);
                pKF->mbHasHessian = true;
                for(int r=0; r<6; ++r)
                {
                    for(int c=0; c<6; ++c)
                    {
                        //cout  << vSE3->hessian(r, c) << ", ";
                        pKF->mHessianPose.at<double>(r, c) = vSE3->hessian(r, c);
                    }
                    //cout << endl;
                }
            }*/

            pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
            pKF->mnBAGlobalForKF = nLoopKF;

            Sophus::SE3f mTwc = pKF->GetPoseInverse();
            Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
            Eigen::Vector3f vector_dist =  mTcGBA_c.translation();
            double dist = vector_dist.norm();

            if(dist > 1)
            {
                //TODO: Luigi here the info about the "outliers" is gathered but it does not seem to be actually used!!!
                int numMonoBadPoints = 0, numMonoOptPoints = 0;
                int numStereoBadPoints = 0, numStereoOptPoints = 0;
                vector<MapPointPtr> vpMonoMPsOpt, vpStereoMPsOpt;
                
#if USE_LINES                  
                int numMonoBadLines = 0, numMonoOptLines = 0;
                int numStereoBadLines = 0, numStereoOptLines = 0;
                vector<MapLinePtr> vpMonoMLsOpt, vpStereoMLsOpt;                
#endif                 

                for(size_t i2=0, iend=vpEdgesMono.size(); i2<iend;i2++)
                {
                    PLVS2::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
                    MapPointPtr pMP = vpMapPointEdgeMono[i2];
                    KeyFramePtr pKFedge = vpEdgeKFMono[i2];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pMP->isBad())
                        continue;

                    if(e->chi2()>5.991 || !e->isDepthPositive())
                    {
                        numMonoBadPoints++;

                    }
                    else
                    {
                        numMonoOptPoints++;
                        vpMonoMPsOpt.push_back(pMP);
                    }

                }

                for(size_t i2=0, iend=vpEdgesStereo.size(); i2<iend;i2++)
                {
                    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
                    MapPointPtr pMP = vpMapPointEdgeStereo[i2];
                    KeyFramePtr pKFedge = vpEdgeKFMono[i2];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pMP->isBad())
                        continue;

                    if(e->chi2()>7.815 || !e->isDepthPositive())
                    {
                        numStereoBadPoints++;
                    }
                    else
                    {
                        numStereoOptPoints++;
                        vpStereoMPsOpt.push_back(pMP);
                    }
                }
                
#if USE_LINES  
                const float chi2LineMono = 5.991;        // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
                const float chi2LineStereo = 9.49;       // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs   
                
                for(size_t i=0, iend=vpLineEdgesMono.size(); i<iend;i++)
                {
                    g2o::EdgeSE3ProjectLine* e = vpLineEdgesMono[i];
                    MapLinePtr  pML = vpMapLineEdgeMono[i];
                    KeyFramePtr pKFedge = vpLineEdgeKFMono[i];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pML->isBad())
                        continue;

                    if(e->chi2()>chi2LineMono || !e->areDepthsPositive())
                    {
                        numMonoBadLines++;

                    }
                    else
                    {
                        numMonoOptLines++;
                        vpMonoMLsOpt.push_back(pML);
                    }

                }

                for(size_t i=0, iend=vpLineEdgesStereo.size(); i<iend;i++)
                {
                    g2o::EdgeSE3ProjectStereoLine* e = vpLineEdgesStereo[i];
                    MapLinePtr  pML = vpMapLineEdgeStereo[i];
                    KeyFramePtr pKFedge = vpLineEdgeKFStereo[i];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pML->isBad())
                        continue;

                    if(e->chi2()>chi2LineStereo || !e->areDepthsPositive())
                    {
                        numStereoBadLines++;
                    }
                    else
                    {
                        numStereoOptLines++;
                        vpStereoMLsOpt.push_back(pML);
                    }
                }                
#endif                 
                
#if USE_OBJECTS_BA
                //TODO: Luigi add object management if something is actually done here!
#endif                 
                
                Verbose::PrintMess("GBA: KF " + to_string(pKF->mnId) + " had been moved " + to_string(dist) + " meters", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--Number of point observations: " + to_string(numMonoOptPoints) + " in mono and " + to_string(numStereoOptPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
                Verbose::PrintMess("--Number of discarded point observations: " + to_string(numMonoBadPoints) + " in mono and " + to_string(numStereoBadPoints) + " in stereo", Verbose::VERBOSITY_DEBUG);
#if USE_LINES  
                if( (vpLineEdgesMono.size()>0) || (vpLineEdgesStereo.size()>0) )
                {
                    Verbose::PrintMess("--Number of line observations: " + to_string(numMonoOptLines) + " in mono and " + to_string(numStereoOptLines) + " in stereo", Verbose::VERBOSITY_DEBUG);
                    Verbose::PrintMess("--Number of line point observations: " + to_string(numMonoBadLines) + " in mono and " + to_string(numStereoBadLines) + " in stereo", Verbose::VERBOSITY_DEBUG);
                }
#endif                 
            }
        }
    }
    Verbose::PrintMess("BA: KFs updated", Verbose::VERBOSITY_DEBUG);

    //Points
    for(size_t i=0; i<vpMPoints.size(); i++)
    {
        if(vbNotIncludedMPoints[i])
            continue;

        MapPointPtr pMP = vpMPoints[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        //if(!vPoint) continue; // redundant 

        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
    
#if USE_LINES_BA      
    // Lines
    for(size_t i=0; i<vpMLines.size(); i++)
    {
        if(vbNotIncludedMLines[i])
            continue;

        MapLinePtr pML = vpMLines[i];

        if(pML->isBad())
            continue;
        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pML->mnId+maxPointId+1));
        //if(!vLine) continue;  // redundant 

        const Eigen::Vector3f pStartNew = (static_cast<const Eigen::Matrix<double,3,1> >(vLine->estimate().head(3))).cast<float>();
        const Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Matrix<double,3,1> >(vLine->estimate().tail(3))).cast<float>();
        
        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pML->SetWorldEndPoints(pStartNew, pEndNew);
            pML->UpdateNormalAndDepth();
        }
        else
        {
            pML->mPosStartGBA = pStartNew;
            pML->mPosEndGBA = pEndNew;   
            pML->mnBAGlobalForKF = nLoopKF;
        }

        if(vLine->isBad()) pML->SetBadFlag();
    }
#endif
    
#if USE_OBJECTS_BA      
    // Objects 
    for(size_t i=0; i<vpMObjects.size(); i++)
    {
        if(vbNotIncludedMObjects[i])
            continue;

        MapObjectPtr pMObj = vpMObjects[i];

        if(pMObj->isBad())
            continue;
        
        g2o::VertexSim3Expmap* vObject = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pMObj->mnId+maxLineId+1));
        //if(!vObject) continue;  // redundant 
        
        g2o::Sim3 g2oSow = vObject->estimate();   // Sow = [Row/s, tow; 0, 1]       
        Eigen::Matrix3d eigRow = g2oSow.rotation().toRotationMatrix();
        Eigen::Vector3d eigtow = g2oSow.translation();
        double scaleow = g2oSow.scale(); // this is equal to 1/s       
        
        Sophus::Sim3f SowNew(Sophus::RxSO3d(scaleow, eigRow).cast<float>(), eigtow.cast<float>());  // Sow = [Row/s, tow; 0, 1] 

        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pMObj->SetSim3Pose(SowNew);
            //pMObj->UpdateNormalAndDepth();
        }
        else
        {
            // pMObj->mSowGBA.create(4,4,CV_32F);
            // SowNew.copyTo(pMObj->mSowGBA);
            pMObj->mSowGBA = SowNew;
            pMObj->mnBAGlobalForKF = nLoopKF;
        }
    }
#endif  

}


void Optimizer::FullInertialBA(Map *pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool *pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd *vSingVal, bool *bHess)
{
#if 1 //VERBOSE_BA
    std::cout << "******************************" << std::endl; 
    std::cout << "Optimizer::FullInertialBA() " << std::endl; 
    std::cout << "******************************" << std::endl; 
#endif

    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPointPtr> vpMPs = pMap->GetAllMapPoints();

#if USE_LINES_FULL_BA_INERTIAL
    vector<MapLinePtr> vpMLs = pMap->GetAllMapLines();
#endif
    
#if USE_OBJECTS_BA
    vector<MapObjectPtr> vpMOs = pMap->GetAllMapObjects();
#endif  

    // Setup optimizer
    g2o::SparseOptimizer optimizer;    
#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFramePtr pIncKF;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF=pKFi;
        bool bFixed = false;
        if(bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF>=(maxKFid-1)) || (pKFi->mnBAFixedForKF>=(maxKFid-1));
            if(!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit)
            {
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid+3*(pKFi->mnId)+2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid+3*(pKFi->mnId)+3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit)
    {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4*maxKFid+2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4*maxKFid+3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if(bFixLocal)
    {
        if(nNonFixed<3)
            return;
    }

    // IMU links
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        if(!pKFi->mPrevKF)
        {
            MSG_WARN_STREAM("NO INERTIAL LINK TO PREVIOUS FRAME - KF id: " << pKFi->mnId);
            continue;
        }

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(pKFi->bImu && pKFi->mPrevKF->bImu)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);

                g2o::HyperGraph::Vertex* VG1;
                g2o::HyperGraph::Vertex* VA1;
                g2o::HyperGraph::Vertex* VG2;
                g2o::HyperGraph::Vertex* VA2;
                if (!bInit)
                {
                    VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
                    VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
                    VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
                    VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);
                }
                else
                {
                    VG1 = optimizer.vertex(4*maxKFid+2);
                    VA1 = optimizer.vertex(4*maxKFid+3);
                }

                g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);

                if (!bInit)
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                        continue;
                    }
                }
                else
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<endl;
                        continue;
                    }
                }

                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit)
                {
                    EdgeGyroRW* egr= new EdgeGyroRW();
                    egr->setVertex(0,VG1);
                    egr->setVertex(1,VG2);
                    Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0,VA1);
                    ear->setVertex(1,VA2);
                    Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
                cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
        }
    }

    if (bInit)
    {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4*maxKFid+2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4*maxKFid+3);

        // Add prior to comon biases
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc* epa = new EdgePriorAcc(bprior);
        epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA; //
        epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro* epg = new EdgePriorGyro(bprior);
        epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG; //
        epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    const float thHuberLineMono = sqrt(5.991);  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    const unsigned long iniMPid = maxKFid*5;

    vector<bool> vbNotIncludedMP(vpMPs.size(),false);

#if USE_LINES_FULL_BA_INERTIAL    
    vector<bool> vbNotIncludedMLines(vpMLs.size(),false);
#endif
    
#if USE_OBJECTS_BA_INERTIAL     
    vector<bool> vbNotIncludedMObjects(vpMOs.size(),false);
#endif   

#define SKIP_BAD_MPS_FULL_INERTIAL 1

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPointPtr pMP = vpMPs[i];
#if SKIP_BAD_MPS_FULL_INERTIAL        
        // Luigi: this seems to be missing here!
        if(pMP->isBad())
            continue;
#endif             
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();


        bool bAllFixed = true;

        //Set edges
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnId>maxKFid)
                continue;

            if(!pKFi->isBad())
            {
                const int leftIndex = get<0>(mit->second);
                cv::KeyPoint kpUn;

                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0) // Monocular observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                }
                else if(leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if(pKFi->mpCamera2){ // Monocular right observation
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                        if(bAllFixed)
                            if(!VP->fixed())
                                bAllFixed=false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if(bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
    }

    const unsigned long maxPointId = iniMPid+1+MapPoint::GetCurrentMaxId(); 

#if USE_LINES_FULL_BA_INERTIAL    // ---------------------------------------------------------
    // Set MapLine vertices
    for(size_t i=0; i<vpMLs.size(); i++)
    {
        const MapLinePtr pML = vpMLs[i];
        if(pML->isBad())
            continue;

        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);             
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());        
        const int id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine);

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();
        //if(observations.size() < kNumMinLineObservationsForBA) continue;        

        bool bAllFixed = true;
        int nEdges = 0;
        
        //Set edges
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKFi = mit->first;
            if(pKFi->isBad() || pKFi->mnId>maxKFid)
                continue;
            
            VertexPose* VP = dynamic_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
            if(VP == NULL)
                continue;              
            if(bAllFixed)
                if(!VP->fixed())
                    bAllFixed=false;

            const ImuCamPose& imuCamPose = VP->estimate();
            
            const int leftIndex = get<0>(mit->second);
            float uRightLineStart = -1;
            float uRightLineEnd = -1;
            if(leftIndex != -1 && !pKFi->mvuRightLineStart.empty()) 
            {
                uRightLineStart = pKFi->mvuRightLineStart[leftIndex];
                uRightLineEnd = pKFi->mvuRightLineEnd[leftIndex];
            }
                            
            // Monocular observation
    #if USE_LINES_STEREO_INERTIAL                
            if( (leftIndex != -1) && ( uRightLineStart<0 || uRightLineEnd<0 ) )
    #endif            
            {
                nEdges++;

                const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[leftIndex];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
            
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                EdgeLineMono* e = new EdgeLineMono(0); // 0 = left cam index
                    
                e->setVertex(0, vertexLine); 
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                e->setMeasurement(obs);          
                
        #if !USE_NEW_LINE_INFORMATION_MAT   
                const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
        #else
                const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];

                Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                Eigen::Vector2d projMapP, projMapQ;
                e->getMapLineProjections(projMapP, projMapQ);
                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
                e->setInformation(Info);
        #endif                 

        #if USE_CAUCHY_KERNEL_FOR_LINES
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberLineMono);
        #endif    
                e->setRobustKernel(rk);

                optimizer.addEdge(e);             
            }
    #if USE_LINES_STEREO_INERTIAL              
            else if ( (leftIndex != -1) && ( uRightLineStart>=0 && uRightLineEnd>=0 ) ) //Stereo observation
            {
                nEdges++;

                const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[leftIndex];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);   

                EdgeLineStereo* e = new EdgeLineStereo(0); // 0 = left cam index

                e->setVertex(0, vertexLine);
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                e->setMeasurement(obs);

                // the following two are actually derived/indirect observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                const Eigen::Vector3f XSc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.startPointX,klUn.startPointY),pKFi->mvDepthLineStart[leftIndex] );
                const Eigen::Vector3f XEc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.endPointX,klUn.endPointY),pKFi->mvDepthLineEnd[leftIndex] );
                //std::cout << "XSc_backproj: " << XSc_backproj.transpose() << ", XEc_backproj: " << XEc_backproj.transpose() << std::endl; 
                e->setBackprojections(XSc_backproj, XEc_backproj);
                e->muWeigth = Optimizer::skMuWeightForLine3dDist;
                
                e->init(); // here we check the match between Bp and P (Bq and Q)
                
        #if !USE_NEW_LINE_INFORMATION_MAT                   
                const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                Info(0,0)*=invSigma2;
                Info(1,1)*=invSigma2;
                Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
        #else
                const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                Eigen::Vector2d projMapP, projMapQ;
                Eigen::Vector3d mapP, mapQ;
                e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                Eigen::Vector3d &backprojP = e->XSbc;
                Eigen::Vector3d &backprojQ = e->XEbc; 

                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
            #if USE_NEW_LINE_INFORMATION_MAT_STEREO                
                Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                sigma2, klUn.octave,
                                pKFi->fx, pKFi->fy, pKFi->mbfInv, 
                                projMapP, projMapQ, 
                                mapP, mapQ,
                                backprojP, backprojQ);    
            #else
                const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
            #endif
                
        #endif
                
                e->setInformation(Info);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberLineStereo);
        #endif    
                e->setRobustKernel(rk);
        
                optimizer.addEdge(e);               
            } 
            // else {
            //     const int rightIndex = get<1>(mit->second);
            //     std::cout << IoColor::Yellow() << "FullInertialBA - unexected line case - leftIndex: " << leftIndex << ", rightIndex: " << rightIndex << ", uRightLineStart: " << uRightLineStart << ", uRightLineEnd: " << uRightLineEnd << std::endl;     
            // }
    #endif  // if USE_LINES_STEREO_INERTIAL               
            
    #if USE_LINES_RIGHT_PROJECTION
            if(pKFi->mpCamera2)
            {
                int rightIndex = get<1>(mit->second);

                if(rightIndex != -1 && rightIndex < pKFi->mvKeyLinesRightUn.size())
                {
                    rightIndex -= pKFi->NlinesLeft;

                    nEdges++;

                    const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesRightUn[rightIndex];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
                    Eigen::Matrix<double,3,1> obs;
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                    EdgeLineMono* e = new EdgeLineMono(1); // 1 = right cam index
                        
                    e->setVertex(0, vertexLine); 
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);          
                    
                    const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);          

            #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(thHuberLineMono);
            #endif    
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);                          
                }
            }
    #endif 

        }

        //if(nEdges<kNumMinLineObservationsForBA)  <- in order to use this one should also remove the edges! 
        if(bAllFixed || nEdges==0)
        {
            optimizer.removeVertex(vLine);
            vbNotIncludedMLines[i]=true;
        }
        else 
        {
            vbNotIncludedMLines[i]=false;
        }
        
        
    } // end for(size_t i=0; i<vpMLines.size(); i++)
#endif
    

    const unsigned long maxLineId = maxPointId+1+MapLine::GetCurrentMaxId();    
    
#if USE_OBJECTS_BA_INERTIAL     // ---------------------------------------------------------
    
    // TODO: update in order to use VertexPose

    // bool bFixScale = false;    
    // const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    // const int nExpectedSizeObjects = vpMObjects.size()*5; // 5 views per object on the average

    // vector<g2o::EdgeSim3SE3*> vpEdgesObject;
    // vpEdgesObject.reserve(nExpectedSizeObjects);
    
    // std::vector<double> vEdgesObjectSquaredErrors;
    // vEdgesObjectSquaredErrors.reserve(nExpectedSizeObjects);
    
    // // Set MapObject vertices
    // for(size_t i=0; i<vpMObjects.size(); i++)
    // {
    //     MapObjectPtr pMObj = vpMObjects[i];
    //     if(pMObj->isBad())
    //         continue;
    //     g2o::VertexSim3Expmap* vObject = new g2o::VertexSim3Expmap();
    //     Eigen::Matrix<double,3,3> Row = pMObj->GetRotation().cast<double>();
    //     Eigen::Matrix<double,3,1> tow = pMObj->GetTranslation().cast<double>();
    //     double objectScale = pMObj->GetScale();
    //     g2o::Sim3 Sow(Row,tow,1./objectScale); // Sow = [Row/s, tow; 0, 1]  
    //     vObject->setEstimate(Sow);
    //     int id = pMObj->mnId+maxLineId+1;
    //     vObject->setId(id);
    //     vObject->setMarginalized(true);
    //     vObject->_fix_scale = bFixScale;        
    //     optimizer.addVertex(vObject);

    //     g2o::OptimizableGraph::Vertex* vertexObject = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
    //     const map<KeyFramePtr,ObjectObservation> observations = pMObj->GetObservations();   

    //     int nEdges = 0;
    //     //SET EDGES
    //     for(map<KeyFramePtr,ObjectObservation>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
    //     {
    //         KeyFramePtr pKFi = mit->first;
    //         if(pKFi->isBad() || pKFi->mnId>maxKFid)
    //             continue;
            
    //         g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));                        
    //         if(vertexKF == NULL)
    //             continue;              

    //         nEdges++;
            
    //         const ObjectObservation& observation = mit->second;

    //         //if(!observation.bFromKF) continue; 

    //         Sophus::SE3f Tko = observation.GetSE3();
    //         Eigen::Matrix<double,3,3> Rko = Tko.rotationMatrix().cast<double>();
    //         Eigen::Matrix<double,3,1> tko = Tko.translation().cast<double>();                
    //         double observedScale = observation.fScale;
                
    //         const g2o::Sim3 Sko(Rko,tko,observedScale); // Sko = [s*Rko, tko; 0, 1]             

    //         g2o::EdgeSim3SE3* e = new g2o::EdgeSim3SE3();
    //         e->setVertex(0, vertexObject);  // Sim3 Sow              
    //         e->setVertex(1, vertexKF); // SE3 Tkw                   
    //         e->setMeasurement(Sko);
    //         e->setInformation(matLambda);

    //         if(bRobust)
    //         {
    //     #if USE_CAUCHY_KERNEL_FOR_OBJECTS
    //             g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
    //     #else 
    //             g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //             rk->setDelta(thHuberObjectTimesSigma);
    //     #endif                 

    //             e->setRobustKernel(rk);
    //         }
            
    //         vpEdgesObject.push_back(e);
            
    //         e->computeError();
    //         vEdgesObjectSquaredErrors.push_back(e->chi2()); // here we accumulate the squared errors (given that matLambda is the identity)
    //         //std::cout << "chi2: " << vEdgesObjectSquaredErrors[vEdgesObjectSquaredErrors.size()-1] << std::endl;            

    //         optimizer.addEdge(e);
   
    //         vpObjectEdges.push_back(e);
    //         vpObjectEdgeKF.push_back(pKFi);
    //         vpMapObjectEdge.push_back(pMObj);
    //     }

    //     if(nEdges==0)
    //     {
    //         optimizer.removeVertex(vObject);
    //         vbNotIncludedMObjects[i]=true;
    //     }
    //     else
    //     {
    //         vbNotIncludedMObjects[i]=false;
    //     }
        
    // } // end for(size_t i=0; i<vpMObjects.size(); i++)
    
    
    // // we need this step in order to set a decent information matrix 
    // if(vEdgesObjectSquaredErrors.size()>0)
    // {
    //     double sigmaEdgesObjectsSquared = Utils::FindSigmaSquared(vEdgesObjectSquaredErrors);   
    //     double invSigmaEdgesObjectsSquared = 1./sigmaEdgesObjectsSquared;
    //     for(size_t i=0, iend=vEdgesObjectSquaredErrors.size(); i<iend;i++)
    //     {
    //         vpEdgesObject[i]->information() *= invSigmaEdgesObjectsSquared;  // N.B.: in this way one has e->chi2() = err_k^2/sigma_err^2
    //     }
    // }    
    
#endif   // USE_OBJECTS_BA_INERTIAL 
    
    //------------------------------------------------------------------------------

    if(pbStopFlag)
        if(*pbStopFlag)
            return;


    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if(nLoopId==0)
        {
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),VP->estimate().tcw[0].cast<float>());
            pKFi->mnBAGlobalForKF = nLoopId;

        }
        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            if(nLoopId==0)
            {
                pKFi->SetVelocity(VV->estimate().cast<float>());
            }
            else
            {
                pKFi->mVwbGBA = VV->estimate().cast<float>();
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            }
            else
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(4*maxKFid+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(4*maxKFid+3));
            }

            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
            if(nLoopId==0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    //Points
    for(size_t i=0; i<vpMPs.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPointPtr pMP = vpMPs[i];
#if SKIP_BAD_MPS_FULL_INERTIAL        
        // Luigi: this seems to be missing here!
        if(pMP->isBad())
            continue;
#endif  

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));

        if(nLoopId==0)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopId;
        }

    }

#if USE_LINES_FULL_BA_INERTIAL   
    //Lines
    for(size_t i=0; i<vpMLs.size(); i++)
    {
        if(vbNotIncludedMLines[i])
            continue;

        MapLinePtr pML = vpMLs[i];
        if(pML->isBad())
            continue;

        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pML->mnId+maxPointId+1));
        //if(!vLine) continue;  // redundant 

        const Eigen::Vector3f pStartNew = (static_cast<const Eigen::Vector3d>(vLine->estimate().head(3))).cast<float>();
        const Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Vector3d>(vLine->estimate().tail(3))).cast<float>();
                
        if(nLoopId==0)
        {
            pML->SetWorldEndPoints(pStartNew, pEndNew);
            pML->UpdateNormalAndDepth();
        }
        else
        {
            pML->mPosStartGBA = pStartNew;
            pML->mPosEndGBA = pEndNew;   
            pML->mnBAGlobalForKF = nLoopId;
        }

        if(vLine->isBad()) pML->SetBadFlag();
    }
#endif    

    pMap->IncreaseChangeIndex();
}


int Optimizer::PoseOptimization(Frame *pFrame)
{

#if VERBOSE_POSE_OPTIMIZATION
    std::cout << "Optimizer::PoseOptimization() " << std::endl; 
#endif  

    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver;
        
#if USE_LINES_POSE_OPTIMIZATION    
    const size_t numLines = pFrame->Nlines;
    
#if USE_BA_VARIABLE_SIZE_SOLVER    
    if(numLines>0)
    {
        
#ifdef USE_G2O_NEW        
        //solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
        //    g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));        
        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>())); // using Cholmod to get covariance below
#else        
        //g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>(); //alternative Eigen solver        
        g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif
        
    }
    else 
#endif   // USE_BA_VARIABLE_SIZE_SOLVER 
    {
        
#ifdef USE_G2O_NEW 
        //solver = new g2o::OptimizationAlgorithmLevenberg(
        //    g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>()));        
        solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>())); 
#else        
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);        
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif
        
    }    
    
#else    
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);    
#endif // USE_LINES_POSE_OPTIMIZATION

    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;
    int nInitialLineCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
    
    //g2o::OptimizableGraph::Vertex* vertexSE3 = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0));   

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<PLVS2::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<PLVS2::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
    vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
    vpEdgesMono.reserve(N);
    vpEdgesMono_FHR.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeRight.reserve(N);

#if !USE_RGBD_POINT_REPROJ_ERR    
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
#else
    vector<g2o::EdgeRgbdSE3ProjectXYZOnlyPose*> vpEdgesStereo;
#endif 
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);    // chi-squared 2 DOFs
    const float deltaStereo = sqrt(7.815);  // chi-squared 3 DOFs
    
#if USE_LINES_POSE_OPTIMIZATION     
    // Set MapLine vertices
    const int Nlines = pFrame->Nlines;
    
    vector<g2o::EdgeSE3ProjectLineOnlyPose*> vpEdgesLineMono;
    vector<size_t> vnIndexEdgeLineMono;
    vpEdgesLineMono.reserve(Nlines);
    vnIndexEdgeLineMono.reserve(Nlines);    
    
    
    vector<g2o::EdgeSE3ProjectStereoLineOnlyPose*> vpEdgesLineStereo;
    vector<size_t> vnIndexEdgeLineStereo;
    vpEdgesLineStereo.reserve(Nlines);
    vnIndexEdgeLineStereo.reserve(Nlines); 

    vector<PLVS2::EdgeSE3ProjectLineOnlyPoseToBody*> vpEdgesLineMonoRight;
    vector<size_t> vnIndexEdgeLineMonoRight;
    vpEdgesLineMonoRight.reserve(Nlines);
    vnIndexEdgeLineMonoRight.reserve(Nlines);    

    const float deltaLineMono   = sqrt(5.991);// chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float deltaLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
        
#endif
    
    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    // start points 
    for(int i=0; i<N; i++)
    {
        MapPointPtr pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            //Conventional SLAM
            if(!pFrame->mpCamera2){
                // Monocular observation
                if(pFrame->mvuRight[i]<0)
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    PLVS2::EdgeSE3ProjectXYZOnlyPose* e = new PLVS2::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    //e->setVertex(0, vertexSE3);
                    
		            e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else  // Stereo observation
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    //SET EDGE
                    Eigen::Matrix<double,3,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
#if !USE_RGBD_POINT_REPROJ_ERR                 
                    const float &kp_ur = pFrame->mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                
                    g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
#else
                    const float &kpDelta = pFrame->mvDepth[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kpDelta;                
                
                    g2o::EdgeRgbdSE3ProjectXYZOnlyPose* e = new g2o::EdgeRgbdSE3ProjectXYZOnlyPose();
#endif 
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    //e->setVertex(0, vertexSE3);
                    
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];

#if !USE_RGBD_POINT_REPROJ_ERR                   
    #if !USE_NEW_STEREO_POINT_INFORMATION_MAT              
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;                
    #else
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Zero(); 
                    SetStereoPointInformationMat(Info, invSigma2, pFrame->mbf, pFrame->mbfInv, pFrame->mvDepth[i]);
    #endif 
#else
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Zero(); 
                    SetRgbdPointInformationMat(Info, invSigma2, pFrame->mbfInv, pFrame->mvDepth[i]);
                
#endif
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
#if !USE_RGBD_POINT_REPROJ_ERR                              
                    e->bf = pFrame->mbf;
#endif        
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
            }
            //SLAM with respect a rigid body
            else{
                nInitialCorrespondences++;

                cv::KeyPoint kpUn;

                if (i < pFrame->Nleft) {    //Left camera observation
                    kpUn = pFrame->mvKeys[i];

                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    PLVS2::EdgeSE3ProjectXYZOnlyPose *e = new PLVS2::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else {   //Right camera observation
                    kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    pFrame->mvbOutlier[i] = false;

                    PLVS2::EdgeSE3ProjectXYZOnlyPoseToBody *e = new PLVS2::EdgeSE3ProjectXYZOnlyPoseToBody();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera2;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    e->mTrl = g2o::SE3Quat(pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(), pFrame->GetRelativePoseTrl().translation().cast<double>());

                    optimizer.addEdge(e);

                    vpEdgesMono_FHR.push_back(e);
                    vnIndexEdgeRight.push_back(i);
                }
            }
        }
    } // end points 
    

#if USE_LINES_POSE_OPTIMIZATION     

    // start lines
    for(int i=0; i<Nlines; i++)
    {
        MapLinePtr pML = pFrame->mvpMapLines[i];
        if(pML)
        {
            
            if( !pFrame->mpCamera2 ||                                  // Convential left image of pinhole cameras 
                ( (pFrame->mpCamera2) && (i < pFrame->NlinesLeft) )    // Left image of fisheye cameras
              )
            {
                // Monocular observation
        #if USE_LINE_STEREO            
                if( (pFrame->mvuRightLineStart.empty()) || (pFrame->mvuRightLineStart[i]<0) || (pFrame->mvuRightLineEnd[i]<0) )
        #endif
                {
                    nInitialLineCorrespondences++;
                    pFrame->mvbLineOutlier[i] = false;
                    pFrame->mvuNumLinePosOptFailures[i] = 0;

                    Eigen::Matrix<double,3,1> obs;
                    const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                    g2o::EdgeSE3ProjectLineOnlyPose* e = new g2o::EdgeSE3ProjectLineOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    //e->setVertex(0, vertexSE3);

                    e->setMeasurement(obs);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;

                    Eigen::Vector3f XSw, XEw;
                    pML->GetWorldEndPoints(XSw, XEw);                   

                    e->XSw[0] = XSw(0);
                    e->XSw[1] = XSw(1);
                    e->XSw[2] = XSw(2);

                    e->XEw[0] = XEw(0);
                    e->XEw[1] = XEw(1);
                    e->XEw[2] = XEw(2);

        #if !USE_NEW_LINE_INFORMATION_MAT   
                    const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
        #else
                    const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave];

                    Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                    Eigen::Vector2d projMapP, projMapQ;
                    e->getMapLineProjections(projMapP, projMapQ);
                    Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2,  
                               klUn.startPointX,klUn.startPointY, 
                               klUn.endPointX,klUn.endPointY, 
                               lineRepresentation.nx, lineRepresentation.ny, 
                               projMapP, projMapQ);
                    e->setInformation(Info);

        #endif

        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(deltaLineMono);
        #endif                                         
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);

                    vpEdgesLineMono.push_back(e);
                    vnIndexEdgeLineMono.push_back(i);
                }
    #if USE_LINE_STEREO      
                else  // Stereo observation
                {
                    nInitialLineCorrespondences++;
                    pFrame->mvbLineOutlier[i] = false;
                    pFrame->mvuNumLinePosOptFailures[i] = 0;

                    Eigen::Matrix<double,3,1> obs;
                    const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                    g2o::EdgeSE3ProjectStereoLineOnlyPose* e = new g2o::EdgeSE3ProjectStereoLineOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    //e->setVertex(0, vertexSE3);

                    e->setMeasurement(obs);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;

                    Eigen::Vector3f XSw, XEw;
                    pML->GetWorldEndPoints(XSw, XEw);                    

                    e->XSw = Eigen::Vector3d(XSw(0), XSw(1), XSw(2));
                    e->XEw = Eigen::Vector3d(XEw(0), XEw(1), XEw(2));  

                    // the following two are actually derived observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                    e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pFrame->mvDepthLineStart[i] );
                    e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pFrame->mvDepthLineEnd[i] );

                    e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 

                    e->init();                 

        #if !USE_NEW_LINE_INFORMATION_MAT                   
                    const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
                    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)
                    const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                
                    Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                    Info(0,0)*=invSigma2;
                    Info(1,1)*=invSigma2;
                    Info(2,2)*=invSigma2PointLineDistance;
                    Info(3,3)*=invSigma2PointLineDistance;
        #else
                    const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave];
                    Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                    Eigen::Vector2d projMapP, projMapQ;
                    Eigen::Vector3d mapP, mapQ;
                    e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                    Eigen::Vector3d &backprojP = e->XSbc;
                    Eigen::Vector3d &backprojQ = e->XEbc; 

                    Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                               klUn.startPointX,klUn.startPointY, 
                               klUn.endPointX,klUn.endPointY, 
                               lineRepresentation.nx, lineRepresentation.ny, 
                               projMapP, projMapQ);
            #if USE_NEW_LINE_INFORMATION_MAT_STEREO                  
                    Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                    sigma2, klUn.octave,
                                    pFrame->fx, pFrame->fy, pFrame->mbfInv,
                                    projMapP, projMapQ, 
                                    mapP, mapQ,
                                    backprojP, backprojQ);          
            #else
                    const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
                    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)       
                    const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                        
                    Info(2,2)=invSigma2PointLineDistance;
                    Info(3,3)=invSigma2PointLineDistance;
            #endif

        #endif                


                    e->setInformation(Info);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(deltaLineStereo);
        #endif   
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);

                    vpEdgesLineStereo.push_back(e);
                    vnIndexEdgeLineStereo.push_back(i);
                }  
    #endif    // #if USE_LINE_STEREO 

            }
            else
            { // Right image SLAM 

                nInitialLineCorrespondences++;
                pFrame->mvbLineOutlier[i] = false;
                pFrame->mvuNumLinePosOptFailures[i] = 0;

                Eigen::Matrix<double,3,1> obs;
                const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesRightUn[i-pFrame->NlinesLeft];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                PLVS2::EdgeSE3ProjectLineOnlyPoseToBody* e = new PLVS2::EdgeSE3ProjectLineOnlyPoseToBody();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                //e->setVertex(0, vertexSE3);
                e->setMeasurement(obs);

                e->pCamera = pFrame->mpCamera2;
                e->mTrl = g2o::SE3Quat(pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(), pFrame->GetRelativePoseTrl().translation().cast<double>());

                Eigen::Vector3f XSw, XEw;
                pML->GetWorldEndPoints(XSw, XEw);                   

                e->XSw[0] = XSw(0);
                e->XSw[1] = XSw(1);
                e->XSw[2] = XSw(2);

                e->XEw[0] = XEw(0);
                e->XEw[1] = XEw(1);
                e->XEw[2] = XEw(2);

                const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

    #if USE_CAUCHY_KERNEL_FOR_LINES
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
    #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(deltaLineMono);
    #endif                                         
                e->setRobustKernel(rk);

                optimizer.addEdge(e);

                vpEdgesLineMonoRight.push_back(e);
                vnIndexEdgeLineMonoRight.push_back(i);
            }
            
        } // if(pML)
        
    } // end lines 

#endif  // USE_LINES_POSE_OPTIMIZATION

    } // end lock MapLine::mGlobalMutex


    if(nInitialCorrespondences + Tracking::sknLineTrackWeigth * nInitialLineCorrespondences < 3)
    {
        std::cout << "Optimizer::PoseOptimization() not enough correspondences " << std::endl; 
        return 0;
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};            // chi-squared 2 DOFs
    const float chi2Stereo[4]={7.815,7.815,7.815,7.815};          // chi-squared 3 DOFs
    const float chi2LineMono[4]={5.991,5.991,5.991,5.991};        // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float chi2LineStereo[4]={9.49,9.49,9.49,9.49};          // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    const int its[4]={10,10,10,10};    

    int nBad=0;
    int nBadLines = 0; 
    for(size_t it=0; it<4; it++)
    {
        Tcw = pFrame->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        nBadLines = 0;
        
        // points  mono 
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            PLVS2::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        // points  mono right 
        for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++)
        {
            PLVS2::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

            const size_t idx = vnIndexEdgeRight[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        // points  stereo
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
#if !USE_RGBD_POINT_REPROJ_ERR              
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];
#else
            g2o::EdgeRgbdSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];
#endif

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

#if !USE_RGBD_POINT_REPROJ_ERR   
            if(chi2>chi2Stereo[it])
#else
            if(chi2>chi2Stereo[it]) // TODO: Luigi check better if a different th is needed here!                
#endif                
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

#if USE_LINES_POSE_OPTIMIZATION           
        
        // lines mono
        for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectLineOnlyPose* e = vpEdgesLineMono[i];

            const size_t idx = vnIndexEdgeLineMono[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineMono[it])
            {                
                //std::cout << "PoseOptimization() - mono line outlier error : " << chi2 << std::endl;
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] +=1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - mono line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        
        // lines mono right
        for(size_t i=0, iend=vpEdgesLineMonoRight.size(); i<iend; i++)
        {
            PLVS2::EdgeSE3ProjectLineOnlyPoseToBody* e = vpEdgesLineMonoRight[i];

            const size_t idx = vnIndexEdgeLineMonoRight[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineMono[it])
            {                
                //std::cout << "PoseOptimization() - mono line outlier error : " << chi2 << std::endl;
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] +=1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - mono line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

#if USE_LINE_STEREO          
        
#if VERBOSE_3D_POINT_LINE_ALIGNMENT_ERROR
        double tot3DSquaredAlignmentError = 0;
        int numStereoLinesInlier = 0; 
#endif        
        // lines stereo
        for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectStereoLineOnlyPose* e = vpEdgesLineStereo[i];

            const size_t idx = vnIndexEdgeLineStereo[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();                    
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineStereo[it])
            {                
                //std::cout << "PoseOptimization() - stereo line outlier error : " << chi2 << std::endl; 
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] += 1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - stereo line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);
                
#if VERBOSE_3D_POINT_LINE_ALIGNMENT_ERROR
                numStereoLinesInlier++;
                tot3DSquaredAlignmentError += e->computeSquared3DError();
#endif                         
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        
#if VERBOSE_3D_POINT_LINE_ALIGNMENT_ERROR
        std::cout << "PoseOptimization() - average 3D point-line alignment error " << sqrt( tot3DSquaredAlignmentError/(2.*numStereoLinesInlier) ) 
                  << ",  num stereo lines inlier: " << numStereoLinesInlier <<  std::endl;        
#endif    
                
#endif // USE_LINE_STEREO
        
#endif // USE_LINES_POSE_OPTIMIZATION   

        if(optimizer.edges().size()<10)
        {
#if VERBOSE_POSE_OPTIMIZATION 
            std::cout << "PoseOptimization() - stop optimization since remained only " << optimizer.edges().size() << " edges" << std::endl;
#endif
            break;
        }
    }    

    
#if VERBOSE_POSE_OPTIMIZATION    
    std::cout << "PoseOptimization() - points: " << nInitialCorrespondences <<" , outliers perc: " << 100*nBad/((float)nInitialCorrespondences) << std::endl; 
#if USE_LINES_POSE_OPTIMIZATION     
    std::cout << "PoseOptimization() - lines: " << nInitialLineCorrespondences <<" , outliers perc: " << 100*nBadLines/((float)nInitialLineCorrespondences) << std::endl; 
#endif
#endif
    
#if PRINT_COVARIANCE     
    /// https://github.com/RainerKuemmerle/g2o/issues/220
    g2o::SparseBlockMatrixX spinv; 
    bool bCovarianceAvailable = optimizer.computeMarginals(spinv,optimizer.vertex(0));
    //The pose covariance is spinv.block(0,0)->eval().
    std::cout << "covariance available: " << bCovarianceAvailable << std::endl;
    if( bCovarianceAvailable )  std::cout << "covariance: " << spinv << std::endl;
#endif
    
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    const g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
            SE3quat_recov.translation().cast<float>());
    pFrame->SetPose(pose);

    return (nInitialCorrespondences-nBad) + Tracking::sknLineTrackWeigth*(nInitialLineCorrespondences-nBadLines);
}


/// < < 

// OK Lines, Ok Objects
void Optimizer::LocalBundleAdjustment(KeyFramePtr pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_MLs, int& num_edges)
{
    
#if VERBOSE_LOCAL_BA
    std::cout << "***********************************" << std::endl; 
    std::cout << "Optimizer::LocalBundleAdjustment() " << std::endl; 
    std::cout << "***********************************" << std::endl; 
#endif    
    
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFramePtr> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const vector<KeyFramePtr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFramePtr pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local map elements seen in Local KeyFrames
    num_fixedKF = 0;
    
    list<MapPointPtr> lLocalMapPoints;
    set<MapPointPtr> sNumObsMP;
        
    list<MapLinePtr> lLocalMapLines;
    set<MapLinePtr> sNumObsML;

    list<MapObjectPtr> lLocalMapObjects;
    set<MapObjectPtr> sNumObsMO;
    
    for(list<KeyFramePtr>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        if(pKFi->mnId==pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
        for(vector<MapPointPtr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPointPtr pMP = *vit;
            if(pMP)
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
                }
        }
        
#if USE_LINES_LOCAL_BA    
        vector<MapLinePtr> vpMLs = pKFi->GetMapLineMatches();
        for(vector<MapLinePtr>::iterator vit=vpMLs.begin(), vend=vpMLs.end(); vit!=vend; vit++)
        {
            MapLinePtr pML = *vit;
            if(pML)
                if(!pML->isBad() && pML->GetMap() == pCurrentMap)
                {

                    if(pML->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapLines.push_back(pML);
                        pML->mnBALocalForKF=pKF->mnId;
                    }

                }
        }        
#endif  
        
#if USE_OBJECTS_LOCAL_BA
        vector<MapObjectPtr> vpMOs = pKFi->GetMapObjectMatches();
        for(vector<MapObjectPtr>::iterator vit=vpMOs.begin(), vend=vpMOs.end(); vit!=vend; vit++)
        {
            MapObjectPtr pMO = *vit;
            if(pMO)
                if(!pMO->isBad() && pMO->GetMap() == pCurrentMap)
                {

                    if(pMO->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapObjects.push_back(pMO);
                        pMO->mnBALocalForKF=pKF->mnId;
                    }

                }
        }
#endif 
    }
    num_MPs = lLocalMapPoints.size();
    num_MLs = lLocalMapLines.size();    

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFramePtr> lFixedCameras;
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId )
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    
#if USE_LINES_LOCAL_BA 
    // Fixed Keyframes. Keyframes that see Local MapLines but that are not Local Keyframes
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }    
#endif
    
#if USE_OBJECTS_LOCAL_BA
    // Fixed Keyframes. Keyframes that see Local MapObjects but that are not Local Keyframes
    for(list<MapObjectPtr>::iterator lit=lLocalMapObjects.begin(), lend=lLocalMapObjects.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,ObjectObservation> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,ObjectObservation>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }        
#endif
    
    num_fixedKF = lFixedCameras.size() + num_fixedKF;

    for(auto it=lFixedCameras.begin(); it!=lFixedCameras.end(); it++)
    {
        //std::cout << "LocalBundleAdjustment - setting KF " << (*it)->mnId << " fixed" << std::endl; 
        (*it)->mnLBACount++;
    }
    
    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
        return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver;
    
#if USE_LINES_LOCAL_BA || USE_OBJECTS_LOCAL_BA 
    const size_t numLines = lLocalMapLines.size();
    const size_t numObjects = lLocalMapObjects.size();    
    
#if USE_BA_VARIABLE_SIZE_SOLVER        
    if( (numLines>0) || (numObjects>0) )
    {
        
#ifdef USE_G2O_NEW        
        solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else  // USE_G2O_NEW       
        g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW  
        
    }
    else
#endif // USE_BA_VARIABLE_SIZE_SOLVER     
    {
        
#ifdef USE_G2O_NEW 
        solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>()));        
#else  // USE_G2O_NEW                
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);        
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW    
        
    }
#else // USE_LINES_LOCAL_BA
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);    
    
#endif // USE_LINES_LOCAL_BA
    
    if (pMap->IsInertial())
        solver->setUserLambdaInit(100.0);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // DEBUG LBA
    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // Set Local KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==pMap->GetInitKFid());
        vSE3->setFixed(pKFi->mbFixed);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = lLocalKeyFrames.size();

    // Set Fixed KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }
    
    unsigned long maxPointId = maxKFid+1+MapPoint::GetCurrentMaxId();     

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<PLVS2::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<PLVS2::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    // point stereo 
#if !USE_RGBD_POINT_REPROJ_ERR      
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
#else
    vector<g2o::EdgeRgbdSE3ProjectXYZ*> vpEdgesStereo;    
#endif
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);


#if USE_LINES_LOCAL_BA         
        
    const int nExpectedSizeLines = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapLines.size();

    // lines mono    
    vector<g2o::EdgeSE3ProjectLine*> vpEdgesLineMono;
    vpEdgesLineMono.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFLineMono; 
    vpEdgeKFLineMono.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeMono; 
    vpMapLineEdgeMono.reserve(nExpectedSizeLines);
    
    // lines stereo
    vector<g2o::EdgeSE3ProjectStereoLine*> vpEdgesLineStereo;
    vpEdgesLineStereo.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFLineStereo;    
    vpEdgeKFLineStereo.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeStereo; 
    vpMapLineEdgeStereo.reserve(nExpectedSizeLines);

    // body lines
    vector<PLVS2::EdgeSE3ProjectLineToBody*> vpEdgesLineBody;
    vpEdgesLineBody.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFLineBody; 
    vpEdgeKFLineBody.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeBody; 
    vpMapLineEdgeBody.reserve(nExpectedSizeLines);
    
#endif // USE_LINES_LOCAL_BA
    
    
#if USE_OBJECTS_LOCAL_BA
    
    unsigned long maxLineId = maxPointId+1+MapLine::GetCurrentMaxId();  

    const int nExpectedSizeObjects = lLocalMapObjects.size()*5; // 5 views per object on average 

    vector<g2o::EdgeSim3SE3*> vpEdgesObject;
    vpEdgesObject.reserve(nExpectedSizeObjects);
    
    vector<KeyFramePtr> vpEdgeKFObject; 
    vpEdgeKFObject.reserve(nExpectedSizeObjects);

    vector<MapObjectPtr > vpMapObjectEdge; 
    vpMapObjectEdge.reserve(nExpectedSizeObjects);    
    
    std::vector<double> vEdgesObjectSquaredErrors;
    vEdgesObjectSquaredErrors.reserve(nExpectedSizeObjects);        
     

#endif // USE_OBJECTS_LOCAL_BA     

    
    const float thHuberMono = sqrt(5.991);      // chi-squared 2 DOFS 
    const float thHuberStereo = sqrt(7.815);    // chi-squared 3 DOFS
    const float thHuberLineMono = sqrt(5.991);  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    
// -----------------------------------------------------------------------------

#if VERBOSE_LOCAL_BA     
    int numConsideredPoints = 0; 
    int numConsideredPointEdges = 0; 
#endif  
    
    int nPoints = 0;

    int nEdges = 0;

    int nKFs = lLocalKeyFrames.size()+lFixedCameras.size();

    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        g2o::OptimizableGraph::Vertex* vertexPoint = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
                
        const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                if(vertexKFi == NULL)
                        continue;
                
                const int leftIndex = get<0>(mit->second);

                // Monocular observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    PLVS2::EdgeSE3ProjectXYZ* e = new PLVS2::EdgeSE3ProjectXYZ();

                    //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setVertex(0, vertexPoint);
                    e->setVertex(1, vertexKFi);                    
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if(leftIndex != -1 && pKFi->mvuRight[leftIndex]>=0)// Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
#if !USE_RGBD_POINT_REPROJ_ERR                      
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
#else
                    const float kpDelta = pKFi->mvDepth[leftIndex];
                    obs << kpUn.pt.x, kpUn.pt.y, kpDelta;

                    g2o::EdgeRgbdSE3ProjectXYZ* e = new g2o::EdgeRgbdSE3ProjectXYZ();                    
#endif
                    //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setVertex(0, vertexPoint);
                    e->setVertex(1, vertexKFi);
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    
#if !USE_RGBD_POINT_REPROJ_ERR                      
    #if !USE_NEW_STEREO_POINT_INFORMATION_MAT              
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
    #else
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Zero();                     
                    SetStereoPointInformationMat(Info, invSigma2, pKFi->mbf, pKFi->mbfInv, pKFi->mvDepth[mit->second]);
    #endif       
#else
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Zero();                     
                    SetRgbdPointInformationMat(Info, invSigma2, pKFi->mbfInv, kpDelta);
#endif          
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
#if !USE_RGBD_POINT_REPROJ_ERR                      
                    e->bf = pKFi->mbf;
#endif
                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        PLVS2::EdgeSE3ProjectXYZToBody *e = new PLVS2::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        const Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }

#if USE_LINES_LOCAL_BA    // ---------------------------------------------------
      
    int numConsideredLines = 0; 
    int numConsideredLineEdges = 0; 
    
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;        
        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);          
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());
        // vLine->P = posStart.cast<double>();
        // vLine->Q = posEnd.cast<double>();
        int id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine); 
        numConsideredLines++;
                
        const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();
        //if(observations.size() < kNumMinLineObservationsForBA)  continue;
        
        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
                
#if CHECK_LINE_VALID_OBSERVATIONS        
        int numValidObservations = 0; 
#endif        
        
        //Set edges
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad() && (pKFi->GetMap() == pCurrentMap))
            {                
                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));                        
                if(vertexKFi == NULL)
                        continue;                
                                    
                const int leftIndex = get<0>(mit->second);
                                    
                // Monocular observation
                if(leftIndex != -1)
                {
                    const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[leftIndex];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);

    #if USE_LINE_STEREO                
                    if( (pKFi->mvuRightLineStart.empty()) || (pKFi->mvuRightLineStart[leftIndex]<0) || (pKFi->mvuRightLineEnd[leftIndex]<0) )
    #endif
                    {
                        Eigen::Matrix<double,3,1> obs;
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                        g2o::EdgeSE3ProjectLine* e = new g2o::EdgeSE3ProjectLine();
                        
                        //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(0, vertexLine);
                        e->setVertex(1, vertexKFi);                    
                        e->setMeasurement(obs);
                        
                        //e->pCamera = pKFi->mpCamera; /// < TODO: Luigi add camera jac management here with mpCamera!
                        
                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;                    

    #if !USE_NEW_LINE_INFORMATION_MAT   
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
    #else
                        const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];

                        Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                        Eigen::Vector2d projMapP, projMapQ;
                        e->getMapLineProjections(projMapP, projMapQ);
                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                        e->setInformation(Info);
    #endif                    
                        
            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineMono);
            #endif                      
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);
                        vpEdgesLineMono.push_back(e);
                        vpEdgeKFLineMono.push_back(pKFi);
                        vpMapLineEdgeMono.push_back(pML);
                    
                        numConsideredLineEdges++;
                
    #if CHECK_LINE_VALID_OBSERVATIONS                        
                        numValidObservations++;
    #endif
                        
                    }
    #if USE_LINE_STEREO                    
                    else // Stereo observation
                    {
                        Eigen::Matrix<double,3,1> obs;
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);   

                        g2o::EdgeSE3ProjectStereoLine* e = new g2o::EdgeSE3ProjectStereoLine();

                        //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(0, vertexLine);
                        e->setVertex(1, vertexKFi); 
                        e->setMeasurement(obs);
                        
                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        
                        // the following two are actually derived observations (using also depth measurements) but we keep them cached inside the edge for simplicity 
                        e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pKFi->mvDepthLineStart[leftIndex]);
                        e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pKFi->mvDepthLineEnd[leftIndex]);
                            
                        e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 
                        e->mu = Optimizer::skMuWeightForLine3dDist;
                        
                        e->init();
                        
    #if !USE_NEW_LINE_INFORMATION_MAT                   
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                    
                        const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;                    
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                        Info(0,0)*=invSigma2;
                        Info(1,1)*=invSigma2;
                        Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                        Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;            
    #else
                        const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                        Eigen::Vector2d projMapP, projMapQ;
                        Eigen::Vector3d mapP, mapQ;
                        e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                        Eigen::Vector3d &backprojP = e->XSbc;
                        Eigen::Vector3d &backprojQ = e->XEbc; 

                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
    #if USE_NEW_LINE_INFORMATION_MAT_STEREO  
                        Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                        sigma2, klUn.octave, 
                                        pKFi->fx, pKFi->fy, pKF->mbfInv, 
                                        projMapP, projMapQ, 
                                        mapP, mapQ,
                                        backprojP, backprojQ);   
    #else
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                      
                        const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;                    
                        Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                        Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
    #endif
                        
    #endif
                        e->setInformation(Info);

            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineStereo);
            #endif   
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);
                        vpEdgesLineStereo.push_back(e);
                        vpEdgeKFLineStereo.push_back(pKFi);
                        vpMapLineEdgeStereo.push_back(pML);
                        
                        numConsideredLineEdges++;
                    
    #if CHECK_LINE_VALID_OBSERVATIONS                        
                        numValidObservations++;
    #endif
                        
                    }
                
                } // end if(leftIndex != -1)
                
                // Monocular observation of Camera 0
                if(pKFi->mpCamera2)
                {
                    int rightIndex = get<1>(mit->second);
                    
                    if(rightIndex != -1 )
                    {                        
                        rightIndex -= pKFi->NlinesLeft;

                        const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesRightUn[rightIndex];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);

                        Eigen::Matrix<double,3,1> obs;
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                        PLVS2::EdgeSE3ProjectLineToBody* e = new PLVS2::EdgeSE3ProjectLineToBody();
                        
                        //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setVertex(0, vertexLine);
                        e->setVertex(1, vertexKFi);                    
                        e->setMeasurement(obs);
                        
                        const Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKFi->mpCamera2;              

                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);          
                        
            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineMono);
            #endif                      
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);
                        vpEdgesLineBody.push_back(e);
                        vpEdgeKFLineBody.push_back(pKFi);
                        vpMapLineEdgeBody.push_back(pML);
                    
                        numConsideredLineEdges++;
                
    #if CHECK_LINE_VALID_OBSERVATIONS                        
                        numValidObservations++;
    #endif
                    }
                    
                }
#endif
            }
            
        }
        
#if CHECK_LINE_VALID_OBSERVATIONS        
        
        if(numValidObservations < kNumMinLineObservationsForBA) ///<- for removing this one should also remove the edges 
        {
            std::cout << "LocalBundleAdjustment - weak line since with only " << numValidObservations << " observations  " << std::endl;
        }
        if(numValidObservations == 0)
        {
            std::cout << "LocalBundleAdjustment - removing invalid line since with zero observations  " << std::endl;
            optimizer.removeVertex(vLine);
        }
#if USE_LINE_PRIOR_BA        
        else
        {
            /// add line prior 
            Eigen::Matrix<double,6,1> obs = Converter::toVector6d(pML->GetWorldPosStart(),pML->GetWorldPosEnd());
            g2o::EdgeLinePrior* e = new g2o::EdgeLinePrior();

            e->setVertex(0, vertexLine);
            e->setMeasurement(obs);
            const float invSigma2 = kInvSigma2PointLinePrior;
            e->setInformation(Eigen::Matrix<double,6,6>::Identity()*invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(kSigmaPointLinePrior);
            
            optimizer.addEdge(e);
        }
#endif // USE_LINE_PRIOR_BA
        
#endif // CHECK_LINE_VALID_OBSERVATIONS        
        
    }
    
#endif // USE_LINES_LOCAL_BA  
    
    
#if USE_OBJECTS_LOCAL_BA  // ---------------------------------------------------
    
    int numConsideredObjects = 0; 
    int numConsideredObjectEdges = 0; 

    bool bFixScale = false;
    
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    for(list<MapObjectPtr >::iterator lit=lLocalMapObjects.begin(), lend=lLocalMapObjects.end(); lit!=lend; lit++)
    {
        MapObjectPtr pMObj = *lit;
        g2o::VertexSim3Expmap* vObject = new g2o::VertexSim3Expmap();
        const Eigen::Matrix<double,3,3> Row = pMObj->GetRotation().cast<double>();
        const Eigen::Matrix<double,3,1> tow = pMObj->GetTranslation().cast<double>();
        const double objectScale = pMObj->GetScale();
        g2o::Sim3 Sow(Row,tow,1./objectScale); // Sow = [Row/s, tow; 0, 1]  
        //std::cout << "LBA - Sow: " << Converter::toCvMat(Sow) << std::endl; 
        vObject->setEstimate(Sow);
        int id = pMObj->mnId+maxLineId+1;
        vObject->setId(id);
        vObject->setMarginalized(true);
        vObject->_fix_scale = bFixScale;        
        optimizer.addVertex(vObject);
             
        numConsideredObjects++;

        g2o::OptimizableGraph::Vertex* vertexObject = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,ObjectObservation> observations = pMObj->GetObservations();

        //Set edges
        for(map<KeyFramePtr,ObjectObservation>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad() && (pKFi->GetMap() == pCurrentMap))
            {                                
                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                if(vertexKFi == NULL) continue;
                                    
                const ObjectObservation& observation = mit->second;
                
                //if(!observation.bFromKF) continue; 
                
                const Sophus::SE3f Tko = observation.GetSE3(); // from object to keyframe               
                const Eigen::Matrix<double,3,3> Rko = Tko.rotationMatrix().cast<double>();
                const Eigen::Matrix<double,3,1> tko = Tko.translation().cast<double>();                
                const double observedScale = observation.fScale;
                
                const g2o::Sim3 Sko(Rko,tko,observedScale); // Sko = [s*Rko, tko; 0, 1]             

                g2o::EdgeSim3SE3* e = new g2o::EdgeSim3SE3();
                e->setVertex(0, vertexObject);  // Sim3   Sow              
                e->setVertex(1, vertexKFi); // SE3        Tkw                   
                e->setMeasurement(Sko);
                e->setInformation(matLambda);
                optimizer.addEdge(e);
                
        #if USE_CAUCHY_KERNEL_FOR_OBJECTS
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberObjectTimesSigma);
        #endif                        

                e->setRobustKernel(rk);
                
                vpEdgesObject.push_back(e);
                vpMapObjectEdge.push_back(pMObj);                
                vpEdgeKFObject.push_back(pKFi);
                
                e->computeError();
                vEdgesObjectSquaredErrors.push_back(e->chi2()); // here we accumulate the squared errors (given that matLambda is the identity)
                
#if VERBOSE_LOCAL_BA                   
                std::cout << "chi2: " << vEdgesObjectSquaredErrors[vEdgesObjectSquaredErrors.size()-1] << std::endl;        
#endif                
                numConsideredObjectEdges++;
            }
        }
         
    }

    // we need this step in order to set a decent information matrix 
    if(vEdgesObjectSquaredErrors.size()>0)
    {
        double sigmaEdgesObjectsSquared = Utils::FindSigmaSquared(vEdgesObjectSquaredErrors); // robust estimation of variance   
        double invSigmaEdgesObjectsSquared = 1./sigmaEdgesObjectsSquared;
        for(size_t i=0, iend=vEdgesObjectSquaredErrors.size(); i<iend;i++)
        {
            vpEdgesObject[i]->information() *= invSigmaEdgesObjectsSquared;  // N.B.: in this way one has e->chi2() = err_k^2/sigma_err^2
        }
    }
    
    
#if VERBOSE_LOCAL_BA     
    std::cout << "LocalBundleAdjustment : objects " << numConsideredObjects <<", obs " << numConsideredObjectEdges << std::endl; 
#endif        
    
#endif // USE_OBJECTS_LOCAL_BA  ------------------------------------------------   
    
    num_edges = nEdges;
    
    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // NOTE: here there is NO outlier removal during optimization as in ORBSLAM2/PLVS!
    //       Below, there is outlier removal after optimization!

    vector<pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());

    int numPointEdgesOutliers = 0; 
    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        PLVS2::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
#if VERBOSE_LOCAL_BA            
            numPointEdgesOutliers++;
#endif            
        }
    }

    for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        PLVS2::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPointPtr pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi,pMP));
#if VERBOSE_LOCAL_BA            
            numPointEdgesOutliers++;
#endif            
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
#if !USE_RGBD_POINT_REPROJ_ERR              
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
#else
        g2o::EdgeRgbdSE3ProjectXYZ* e = vpEdgesStereo[i];
#endif 
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
#if VERBOSE_LOCAL_BA            
            numPointEdgesOutliers++;
#endif
        }
    }


#if USE_LINES_LOCAL_BA
    int numLineEdgesOutliers = 0; 
    
    vector<pair<KeyFramePtr,MapLinePtr> > vLineToErase;
    vLineToErase.reserve(vpEdgesLineMono.size()+vpEdgesLineStereo.size());

    // Check inlier observations for lines  
    for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectLine* e = vpEdgesLineMono[i];
        MapLinePtr pML = vpMapLineEdgeMono[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>5.991 || !e->areDepthsPositive())
        {
            //pML->muNumLineBAFailures++; 
            //if(pML->muNumLineBAFailures < kNumMinBAFailuresForErasingMapLine) continue; 
            
            KeyFramePtr pKFi = vpEdgeKFLineMono[i];
            vLineToErase.push_back(make_pair(pKFi,pML));
            /*
            std::cout << "removed obs " << pML->mnId << std::endl;
            std::cout << "error: " << e->chi2() << std::endl;
            std::cout << "depth start: " << e->depthStart() << std::endl;
            std::cout << "depth end: " << e->depthEnd() << std::endl;
             */
            numLineEdgesOutliers++;
        }
        /*else
        {
            pML->muNumLineBAFailures = 0; 
        }*/
    } 
    
    for(size_t i=0, iend=vpEdgesLineBody.size(); i<iend;i++)
    {
        PLVS2::EdgeSE3ProjectLineToBody* e = vpEdgesLineBody[i];
        MapLinePtr pML = vpMapLineEdgeBody[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>5.991 || !e->areDepthsPositive())
        {
            //pML->muNumLineBAFailures++; 
            //if(pML->muNumLineBAFailures < kNumMinBAFailuresForErasingMapLine) continue; 
            
            KeyFramePtr pKFi = vpEdgeKFLineBody[i];
            vLineToErase.push_back(make_pair(pKFi,pML));
            /*
            std::cout << "removed obs " << pML->mnId << std::endl;
            std::cout << "error: " << e->chi2() << std::endl;
            std::cout << "depth start: " << e->depthStart() << std::endl;
            std::cout << "depth end: " << e->depthEnd() << std::endl;
             */
            numLineEdgesOutliers++;
        }
        /*else
        {
            pML->muNumLineBAFailures = 0; 
        }*/
    } 

#if USE_LINE_STEREO  

#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
    int num3DObservartionInliers = 0; 
    double totSquaredAlignmentError = 0; 
    double totSquaredEndPointsDeviations = 0; 
#endif
    
    for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectStereoLine* e = vpEdgesLineStereo[i];
        MapLinePtr pML = vpMapLineEdgeStereo[i];

        if(pML->isBad())
            continue;
       
        //if(e->chi2()>9.49 || !e->areDepthsPositive())     
        // N.B.: outliers are identified by just considering how close is the representative 3D line to the back-projected measured 2D line  
        //       without considering (mu=0) how much the representative 3D points Q,P are close to the back-projected Qi,Pi from the measured 2D points pi,qi        
        if(e->zeroMuChi2()>9.49 || !e->areDepthsPositive())                   
        {
            //pML->muNumLineBAFailures++; 
            //if(pML->muNumLineBAFailures < kNumMinBAFailuresForErasingMapLine) continue; 
             
            KeyFramePtr pKFi = vpEdgeKFLineStereo[i];
            vLineToErase.push_back(make_pair(pKFi,pML));
            /*
            std::cout << "removed obs " << pML->mnId << std::endl;
            std::cout << "error: " << e->chi2() << std::endl;
            std::cout << "depth start: " << e->depthStart() << std::endl;
            std::cout << "depth end: " << e->depthEnd() << std::endl;
             */
            numLineEdgesOutliers++;
        }
        else
        {
            //pML->muNumLineBAFailures = 0; 
            
#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
            num3DObservartionInliers++; 
            totSquaredAlignmentError += e->computeSquared3DError(); 
            totSquaredEndPointsDeviations += e->computeSquaredEndPointsDeviations();
#endif            
        }
    }  
    
#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
    const int TwoNum3DObservartionInliers = 2*num3DObservartionInliers;
    std::cout << "Local BA - average 3D line error: " << sqrt( totSquaredAlignmentError/TwoNum3DObservartionInliers ) 
              << ", average endpoints deviation: " << sqrt( totSquaredEndPointsDeviations/TwoNum3DObservartionInliers ) 
              << " , num 3D obs inliers: " << num3DObservartionInliers <<  std::endl; 
#endif       
    
#endif // USE_LINE_STEREO
    
#endif // USE_LINES_LOCAL_BA
    
    

#if USE_OBJECTS_LOCAL_BA
    
    int numObjectEdgesOutliers = 0; 
    
    vector<pair<KeyFramePtr,MapObjectPtr > > vObjectToErase;
    vObjectToErase.reserve(vpEdgesObject.size());

    // Check inlier observations for lines  
    for(size_t i=0, iend=vpEdgesObject.size(); i<iend;i++)
    {
        g2o::EdgeSim3SE3* e = vpEdgesObject[i];
        MapObjectPtr pMObj = vpMapObjectEdge[i];

        if(pMObj->isBad())
            continue;

        if(e->chi2()>3)  // err_k^2/sigma_err^2 > 3
        {            
            KeyFramePtr pKFi = vpEdgeKFObject[i];
            vObjectToErase.push_back(make_pair(pKFi, pMObj));
#if VERBOSE_OBJECTS_CHISQUARES            
            std::cout << "removed obs (" << pKFi->mnId << ", " << pMObj->mnId <<")" << std::endl;
#endif
            
            numObjectEdgesOutliers++;
        }
#if VERBOSE_OBJECTS_CHISQUARES        
        std::cout << "error: " << e->chi2() << std::endl;
#endif
    } 
    
#endif // USE_OBJECTS_LOCAL_BA
    

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }


#if USE_LINES_LOCAL_BA
    if(!vLineToErase.empty())
    {
        for(size_t i=0;i<vLineToErase.size();i++)
        {
            KeyFramePtr pKFi = vLineToErase[i].first;
            MapLinePtr pMLi = vLineToErase[i].second;
            pKFi->EraseMapLineMatch(pMLi);
            pMLi->EraseObservation(pKFi);
        }
    }    
#endif    
    
#if USE_OBJECTS_LOCAL_BA
    if(!vObjectToErase.empty())
    {
        for(size_t i=0;i<vObjectToErase.size();i++)
        {
            KeyFramePtr pKFi = vObjectToErase[i].first;
            MapObjectPtr pMOi = vObjectToErase[i].second;
            pKFi->EraseMapObjectMatch(pMOi);
            pMOi->EraseObservation(pKFi);
        }
    }    
#endif        

    // Recover optimized data

    //Keyframes
    for(list<KeyFramePtr>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
        pKFi->SetPose(Tiw);
        pKFi->mnLBACount++;
        //std::cout << "LocalBundleAdjustement() - adjusted KF " << pKFi->mnId << " (fixed: "<< pKFi->mbFixed << ")"<< std::endl; 
    }

    //Points
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

        
#if USE_LINES_LOCAL_BA    
    //Lines
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;        
        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pML->mnId+maxPointId+1));
        //if(vLine==NULL) continue; // check if we actually inserted the line in the graph 
        const Eigen::Matrix<double,6,1> line(vLine->estimate());
        //const cv::Mat pStartNew = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(line.head(3)));
        //const cv::Mat pEndNew   = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(line.tail(3)));
        Eigen::Vector3f pStartNew = (static_cast<const Eigen::Matrix<double,3,1> >(line.head(3))).cast<float>();
        Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Matrix<double,3,1> >(line.tail(3))).cast<float>();        

        pML->SetWorldEndPoints(pStartNew, pEndNew);
        pML->UpdateNormalAndDepth();

        if(vLine->isBad()) pML->SetBadFlag();
    }
#endif    
     
#if USE_OBJECTS_LOCAL_BA    
    //Objects
    for(list<MapObjectPtr >::iterator lit=lLocalMapObjects.begin(), lend=lLocalMapObjects.end(); lit!=lend; lit++)
    {
        MapObjectPtr pMObj = *lit;        
        g2o::VertexSim3Expmap* vObject = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pMObj->mnId+maxLineId+1));
        if(vObject==NULL) continue; // check if we actually inserted the object in the graph 
        g2o::Sim3 correctedSow = vObject->estimate();   // Sow = [Row/s, tow; 0, 1]       
        Eigen::Matrix3d eigRow = correctedSow.rotation().toRotationMatrix();
        Eigen::Vector3d eigtow = correctedSow.translation();
        double scaleow = correctedSow.scale();

        Sophus::Sim3f Sow(Sophus::RxSO3d(scaleow, eigRow).cast<float>(), eigtow.cast<float>()); // Sow = [Row/s, tow; 0, 1] 
        //std::cout << "LBA - resulting Sow: " << Sow << std::endl; 

        pMObj->SetSim3Pose(Sow);

    }
#endif  
    
#if VERBOSE_LOCAL_BA
    
    std::cout << "LocalBundleAdjustment : points " << numConsideredPoints <<" , obs " << numConsideredPointEdges << ", erased obs: " << 100*((float)numPointEdgesOutliers)/numConsideredPointEdges << "%" << std::endl; 
#if USE_LINES_LOCAL_BA     
    std::cout << "LocalBundleAdjustment : lines " << numConsideredLines <<", obs " << numConsideredLineEdges << ", erased obs: " <<  100*((float)numLineEdgesOutliers)/numConsideredLineEdges <<"%" << std::endl;  
#endif
#if USE_OBJECTS_LOCAL_BA     
    std::cout << "LocalBundleAdjustment : objects " << numConsideredObjects <<", obs " << numConsideredObjectEdges << ", erased obs: " <<  100*((float)numObjectEdgesOutliers)/numConsideredObjectEdges <<"%" << std::endl;  
#endif  
    
#endif // VERBOSE_LOCAL_BA    

    pMap->IncreaseChangeIndex();
}

// Ok lines, Ok objects
void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr, set<KeyFramePtr> > &LoopConnections, const bool &bFixScale)
{
#if VERBOSE
    std::cout << "Optimizer::OptimizeEssentialGraph() " << std::endl; 
#endif  
    
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::OptimizationAlgorithmLevenberg* solver;
    
#ifdef USE_G2O_NEW        
    solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_7_3>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>()));        
#else    
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPointPtr> vpMPs = pMap->GetAllMapPoints();
#if USE_LINES_EG    
    const vector<MapLinePtr> vpMLines = pMap->GetAllMapLines();    
#endif
#if USE_OBJECTS_EG
    const vector<MapObjectPtr> vpMOs= pMap->GetAllMapObjects();
#endif 

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    vector<Eigen::Vector3d> vZvectors(nMaxKFid+1); // For debugging
    Eigen::Vector3d z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFramePtr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF->mnId==pMap->GetInitKFid())
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);
        vZvectors[nIDi]=vScw[nIDi].rotation()*z_vec; // For debugging

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    int count_loop = 0;
    for(map<KeyFramePtr, set<KeyFramePtr> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFramePtr> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFramePtr>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);
            count_loop++;
            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    int count_spa_tree = 0;
    int count_cov = 0;
    int count_imu = 0;
    int count_kf = 0;
    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        count_kf = 0;
        KeyFramePtr pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFramePtr pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            count_kf++;
            count_spa_tree++;
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFramePtr> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFramePtr>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFramePtr pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
                count_kf++;
                count_loop++;
            }
        }

        // Covisibility graph edges
        const vector<KeyFramePtr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFramePtr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFramePtr pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/)
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                    count_kf++;
                    count_cov++;
                }
            }
        }

        // Inertial edges if inertial
        if(pKF->bImu && pKF->mPrevKF)
        {
            g2o::Sim3 Spw;
            LoopClosing::KeyFrameAndPose::const_iterator itp = NonCorrectedSim3.find(pKF->mPrevKF);
            if(itp!=NonCorrectedSim3.end())
                Spw = itp->second;
            else
                Spw = vScw[pKF->mPrevKF->mnId];

            g2o::Sim3 Spi = Spw * Swi;
            g2o::EdgeSim3* ep = new g2o::EdgeSim3();
            ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mPrevKF->mnId)));
            ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            ep->setMeasurement(Spi);
            ep->information() = matLambda;
            optimizer.addEdge(ep);
            count_kf++;
            count_imu++;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();

        Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(), CorrectedSiw.translation().cast<float>() / s);
        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPointPtr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();
            if(!pRefKF) continue;             
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }
    
#if USE_LINES_EG      
    // Correct lines. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMLines.size(); i<iend; i++)
    {
        MapLinePtr pML = vpMLines[i];

        if(pML->isBad())
            continue;

        int nIDr;
        if(pML->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pML->mnCorrectedReference;
        }
        else
        {
            KeyFramePtr pRefKF = pML->GetReferenceKeyFrame();
            if(!pRefKF) continue; 
            nIDr = pRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Vector3f P3DSw, P3DEw;
        pML->GetWorldEndPoints(P3DSw, P3DEw);             
        
        Eigen::Matrix<double,3,1> eigP3DSw = P3DSw.cast<double>();
        Eigen::Matrix<double,3,1> eigP3DEw = P3DEw.cast<double>();
        
        Eigen::Matrix<double,3,1> eigCorrectedP3DSw = correctedSwr.map(Srw.map(eigP3DSw));
        Eigen::Matrix<double,3,1> eigCorrectedP3DEw = correctedSwr.map(Srw.map(eigP3DEw));
        
        Eigen::Vector3f correctedP3DSw = eigCorrectedP3DSw.cast<float>();
        Eigen::Vector3f correctedP3DEw = eigCorrectedP3DEw.cast<float>();
        
        pML->SetWorldEndPoints(correctedP3DSw, correctedP3DEw);        
        
        pML->UpdateNormalAndDepth();
    }
#endif


#if USE_OBJECTS_EG
    // Correct objects. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMOs.size(); i<iend; i++)
    {
        MapObjectPtr pMO = vpMOs[i];

        if(pMO->isBad())
            continue;

        int nIDr;
        if(pMO->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMO->mnCorrectedReference;
        }
        else
        {
            KeyFramePtr pRefKF = pMO->GetReferenceKeyFrame();
            if(!pRefKF) continue; 
            nIDr = pRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];
        
        // Map to non-corrected camera (convert Sim(3) Srw to an SE(3) transformation)
        const double scw = Srw.scale();
        Eigen::Matrix3f Rcw = Srw.rotation().matrix().cast<float>();
        Eigen::Vector3f tcw = (Srw.translation()/scw).cast<float>();
        
        const double scale = pMO->GetScale();
        Eigen::Matrix3f Rwo = pMO->GetInverseRotation();
        Eigen::Vector3f two = pMO->GetInverseTranslation();

        const Eigen::Matrix3f Rco = Rcw*Rwo;
        const Eigen::Vector3f tco = Rcw*two+tcw;  

        // Backproject using corrected camera (convert Sim(3) correctedSwr to an SE(3) transformation)
        const double swc = correctedSwr.scale();        
        Eigen::Matrix3f Rwc = correctedSwr.rotation().matrix().cast<float>();
        Eigen::Vector3f twc = (correctedSwr.translation()/swc).cast<float>();
        
        const Eigen::Matrix3f RwoNew = Rwc*Rco;
        const Eigen::Vector3f twoNew = Rwc*tco+twc;
        
        pMO->SetSim3InversePose(RwoNew, twoNew, scale);   // keep the original object to world scale  
    }
#endif 
    
    // TODO Check this changeindex
    pMap->IncreaseChangeIndex();
}

// Ok lines
void Optimizer::OptimizeEssentialGraph(KeyFramePtr pCurKF, vector<KeyFramePtr> &vpFixedKFs, vector<KeyFramePtr> &vpFixedCorrectedKFs,
                                       vector<KeyFramePtr> &vpNonFixedKFs, vector<MapPointPtr> &vpNonCorrectedMPs, 
                                       vector<MapLinePtr> &vpNonCorrectedMLs, vector<MapObjectPtr> &vpNonCorrectedMOs)
{
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpFixedKFs.size()) + " KFs fixed in the merged map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpFixedCorrectedKFs.size()) + " KFs fixed in the old map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonFixedKFs.size()) + " KFs non-fixed in the merged map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonCorrectedMPs.size()) + " MPs non-corrected in the merged map", Verbose::VERBOSITY_DEBUG);
#if USE_LINES_EG  
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonCorrectedMLs.size()) + " MLs non-corrected in the merged map", Verbose::VERBOSITY_DEBUG);
#endif     
#if USE_OBJECTS_EG
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonCorrectedMOs.size()) + " MOs non-corrected in the merged map", Verbose::VERBOSITY_DEBUG);
#endif 
    
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::OptimizationAlgorithmLevenberg* solver;
    
#ifdef USE_G2O_NEW        
    solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_7_3>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>()));        
#else    
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    Map* pMap = pCurKF->GetMap();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    vector<bool> vpGoodPose(nMaxKFid+1);
    vector<bool> vpBadPose(nMaxKFid+1);

    const int minFeat = 100;

    for(KeyFramePtr pKFi : vpFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vCorrectedSwc[nIDi]=Siw.inverse();
        VSim3->setEstimate(Siw);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = false;
    }
    Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded", Verbose::VERBOSITY_DEBUG);

    set<unsigned long> sIdKF;
    for(KeyFramePtr pKFi : vpFixedCorrectedKFs)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vCorrectedSwc[nIDi]=Siw.inverse();
        VSim3->setEstimate(Siw);

        Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
        vScw[nIDi] = g2o::Sim3(Tcw_bef.unit_quaternion(),Tcw_bef.translation(),1.0);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = true;
    }

    for(KeyFramePtr pKFi : vpNonFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        if(sIdKF.count(nIDi)) // It has already added in the corrected merge KFs
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        VSim3->setFixed(false);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = false;
        vpBadPose[nIDi] = true;
    }

    vector<KeyFramePtr> vpKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
    vpKFs.insert(vpKFs.end(),vpFixedKFs.begin(),vpFixedKFs.end());
    vpKFs.insert(vpKFs.end(),vpFixedCorrectedKFs.begin(),vpFixedCorrectedKFs.end());
    vpKFs.insert(vpKFs.end(),vpNonFixedKFs.begin(),vpNonFixedKFs.end());
    set<KeyFramePtr> spKFs(vpKFs.begin(), vpKFs.end());

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    for(KeyFramePtr pKFi : vpKFs)
    {
        int num_connections = 0;
        const int nIDi = pKFi->mnId;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if(vpGoodPose[nIDi])
            correctedSwi = vCorrectedSwc[nIDi];
        if(vpBadPose[nIDi])
            Swi = vScw[nIDi].inverse();

        KeyFramePtr pParentKFi = pKFi->GetParent();

        // Spanning tree edge
        if(pParentKFi && spKFs.find(pParentKFi) != spKFs.end())
        {
            int nIDj = pParentKFi->mnId;

            g2o::Sim3 Sjw;
            bool bHasRelation = false;

            if(vpGoodPose[nIDi] && vpGoodPose[nIDj])
            {
                Sjw = vCorrectedSwc[nIDj].inverse();
                bHasRelation = true;
            }
            else if(vpBadPose[nIDi] && vpBadPose[nIDj])
            {
                Sjw = vScw[nIDj];
                bHasRelation = true;
            }

            if(bHasRelation)
            {
                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3* e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
                num_connections++;
            }

        }

        // Loop edges
        const set<KeyFramePtr> sLoopEdges = pKFi->GetLoopEdges();
        for(set<KeyFramePtr>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFramePtr pLKF = *sit;
            if(spKFs.find(pLKF) != spKFs.end() && pLKF->mnId<pKFi->mnId)
            {
                g2o::Sim3 Slw;
                bool bHasRelation = false;

                if(vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId])
                {
                    Slw = vCorrectedSwc[pLKF->mnId].inverse();
                    bHasRelation = true;
                }
                else if(vpBadPose[nIDi] && vpBadPose[pLKF->mnId])
                {
                    Slw = vScw[pLKF->mnId];
                    bHasRelation = true;
                }


                if(bHasRelation)
                {
                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3* el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                    num_connections++;
                }
            }
        }

        // Covisibility graph edges
        const vector<KeyFramePtr> vpConnectedKFs = pKFi->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFramePtr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFramePtr pKFn = *vit;
            if(pKFn && pKFn!=pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end())
            {
                if(!pKFn->isBad() && pKFn->mnId<pKFi->mnId)
                {

                    g2o::Sim3 Snw =  vScw[pKFn->mnId];
                    bool bHasRelation = false;

                    if(vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId])
                    {
                        Snw = vCorrectedSwc[pKFn->mnId].inverse();
                        bHasRelation = true;
                    }
                    else if(vpBadPose[nIDi] && vpBadPose[pKFn->mnId])
                    {
                        Snw = vScw[pKFn->mnId];
                        bHasRelation = true;
                    }

                    if(bHasRelation)
                    {
                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3* en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                        num_connections++;
                    }
                }
            }
        }

        if(num_connections == 0 )
        {
            Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections", Verbose::VERBOSITY_DEBUG);
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(KeyFramePtr pKFi : vpNonFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();
        Sophus::SE3d Tiw(CorrectedSiw.rotation(),CorrectedSiw.translation() / s);

        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(MapPointPtr pMPi : vpNonCorrectedMPs)
    {
        if(pMPi->isBad())
            continue;

        KeyFramePtr pRefKF = pMPi->GetReferenceKeyFrame();
        while(pRefKF->isBad())
        {
            if(!pRefKF)
            {
                Verbose::PrintMess("MP " + to_string(pMPi->mnId) + " without a valid reference KF", Verbose::VERBOSITY_DEBUG);
                break;
            }

            pMPi->EraseObservation(pRefKF);
            pRefKF = pMPi->GetReferenceKeyFrame();
        }

        if(vpBadPose[pRefKF->mnId])
        {
            Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
            Sophus::SE3f Twr = pRefKF->GetPoseInverse();

            Eigen::Vector3f eigCorrectedP3Dw = Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
            pMPi->SetWorldPos(eigCorrectedP3Dw);

            pMPi->UpdateNormalAndDepth();
        }
        else
        {
            cout << "ERROR: MapPoint has a reference KF from another map" << endl;
        }

    }

#if USE_LINES_EG 
    // Correct lines. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(MapLinePtr pMLi : vpNonCorrectedMLs)
    {
        if(pMLi->isBad())
            continue;

        KeyFramePtr pRefKF = pMLi->GetReferenceKeyFrame();
        while(pRefKF->isBad())
        {
            if(!pRefKF)
            {
                Verbose::PrintMess("ML " + to_string(pMLi->mnId) + " without a valid reference KF", Verbose::VERBOSITY_DEBUG);
                break;
            }

            pMLi->EraseObservation(pRefKF);
            pRefKF = pMLi->GetReferenceKeyFrame();
        }

        if(vpBadPose[pRefKF->mnId])
        {
            Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
            Sophus::SE3f Twr = pRefKF->GetPoseInverse();

            Eigen::Vector3f P3DSw, P3DEw;
            pMLi->GetWorldEndPoints(P3DSw, P3DEw);   

            Eigen::Vector3f correctedP3DSw = Twr * TNonCorrectedwr.inverse() * P3DSw;
            Eigen::Vector3f correctedP3DEw = Twr * TNonCorrectedwr.inverse() * P3DEw;
    
            pMLi->SetWorldEndPoints(correctedP3DSw, correctedP3DEw);  

            pMLi->UpdateNormalAndDepth();
        }
        else
        {
            cout << "ERROR: MapLine has a reference KF from another map" << endl;
        }

    }
#endif

#if USE_OBJECTS_EG
    // Correct objects. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(MapObjectPtr pMOi : vpNonCorrectedMOs)
    {
        if(pMOi->isBad())
            continue;

        KeyFramePtr pRefKF = pMOi->GetReferenceKeyFrame();
        while(pRefKF->isBad())
        {
            if(!pRefKF)
            {
                Verbose::PrintMess("MO " + to_string(pMOi->mnId) + " without a valid reference KF", Verbose::VERBOSITY_DEBUG);
                break;
            }

            pMOi->EraseObservation(pRefKF);
            pRefKF = pMOi->GetReferenceKeyFrame();
        }

        if(vpBadPose[pRefKF->mnId])
        {
            const Sophus::SE3f TNonCorrectedrw = pRefKF->mTwcBefMerge.inverse();
            const Sophus::SE3f Twr = pRefKF->GetPoseInverse();
            
            // Map to non-corrected camera 
            Eigen::Matrix3f Rcw = TNonCorrectedrw.rotationMatrix();
            Eigen::Vector3f tcw = TNonCorrectedrw.translation();
            
            const double scale = pMOi->GetScale();
            Eigen::Matrix3f Rwo = pMOi->GetInverseRotation();
            Eigen::Vector3f two = pMOi->GetInverseTranslation();

            const Eigen::Matrix3f Rco = Rcw*Rwo;
            const Eigen::Vector3f tco = Rcw*two+tcw;  

            // Backproject using corrected camera 
            Eigen::Matrix3f Rwc = Twr.rotationMatrix();
            Eigen::Vector3f twc = Twr.translation();
            
            const Eigen::Matrix3f RwoNew = Rwc*Rco;
            const Eigen::Vector3f twoNew = Rwc*tco+twc;
            
            pMOi->SetSim3InversePose(RwoNew, twoNew, scale);   // keep the original object to world scale  

        }
        else
        {
            cout << "ERROR: MapObject has a reference KF from another map" << endl;
        }

    }
#endif 

}

int Optimizer::OptimizeSim3(KeyFramePtr pKF1, KeyFramePtr pKF2, vector<MapPointPtr> &vpMatches1, g2o::Sim3 &g2oS12, const float th2,
                            const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints)
{
    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    optimizer.setAlgorithm(solver);

    // Camera poses
    const Eigen::Matrix3f R1w = pKF1->GetRotation();
    const Eigen::Vector3f t1w = pKF1->GetTranslation();
    const Eigen::Matrix3f R2w = pKF2->GetRotation();
    const Eigen::Vector3f t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    PLVS2::VertexSim3Expmap * vSim3 = new PLVS2::VertexSim3Expmap();
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->pCamera1 = pKF1->mpCamera;
    vSim3->pCamera2 = pKF2->mpCamera;
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPointPtr> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<PLVS2::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<PLVS2::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;
    vector<bool> vbIsInKF2;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);
    vbIsInKF2.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;
    int nBadMPs = 0;
    int nInKF2 = 0;
    int nOutKF2 = 0;
    int nMatchWithoutMP = 0;

    vector<int> vIdsOnlyInKF2;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPointPtr pMP1 = vpMapPoints1[i];
        MapPointPtr pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

        Eigen::Vector3f P3D1c;
        Eigen::Vector3f P3D2c;

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
                P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(P3D1c.cast<double>());
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
            {
                nBadMPs++;
                continue;
            }
        }
        else
        {
            nMatchWithoutMP++;

            //TODO The 3D position in KF1 doesn't exist
            if(!pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);

                vIdsOnlyInKF2.push_back(id2);
            }
            continue;
        }

        if(i2<0 && !bAllPoints)
        {
            Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints), Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if(P3D2c(2) < 0)
        {
            Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        PLVS2::EdgeSim3ProjectXYZ* e12 = new PLVS2::EdgeSim3ProjectXYZ();

        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        cv::KeyPoint kpUn2;
        bool inKF2;
        if(i2 >= 0)
        {
            kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;
            inKF2 = true;

            nInKF2++;
        }
        else
        {
            float invz = 1/P3D2c(2);
            float x = P3D2c(0)*invz;
            float y = P3D2c(1)*invz;

            obs2 << x, y;
            kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

            inKF2 = false;
            nOutKF2++;
        }

        PLVS2::EdgeInverseSim3ProjectXYZ* e21 = new PLVS2::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);

        vbIsInKF2.push_back(inKF2);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    int nBadOutKF2 = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        PLVS2::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        PLVS2::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPointPtr>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<PLVS2::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<PLVS2::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;

            if(!vbIsInKF2[i])
            {
                nBadOutKF2++;
            }
            continue;
        }

        //Check if remove the robust adjustment improve the result
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        PLVS2::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        PLVS2::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        e12->computeError();
        e21->computeError();

        if(e12->chi2()>th2 || e21->chi2()>th2){
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPointPtr>(NULL);
        }
        else{
            nIn++;
        }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

// Ok lines
void Optimizer::LocalInertialBA(KeyFramePtr pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)
{
#if VERBOSE_BA
    std::cout << "******************************" << std::endl; 
    std::cout << "Optimizer::LocalInertialBA() " << std::endl; 
    std::cout << "******************************" << std::endl; 
#endif     
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt=10;
    int opt_it=10;
    if(bLarge)
    {
        maxOpt=25;
        opt_it=4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap()-2,maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFramePtr> vpOptimizableKFs;
    const vector<KeyFramePtr> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    list<KeyFramePtr> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPointPtr> lLocalMapPoints;
#if USE_LINES_LOCAL_BA_INERTIAL    
    // Optimizable lines seen by temporal optimizable keyframes
    list<MapLinePtr> lLocalMapLines;    
#endif 

    for(int i=0; i<N; i++)
    {
        vector<MapPointPtr> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPointPtr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPointPtr pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }

#if USE_LINES_LOCAL_BA_INERTIAL
        vector<MapLinePtr> vpMLs = vpOptimizableKFs[i]->GetMapLineMatches();
        for(vector<MapLinePtr>::iterator vit=vpMLs.begin(), vend=vpMLs.end(); vit!=vend; vit++)
        {
            MapLinePtr pML = *vit;
            if(pML)
                if(!pML->isBad())
                    if(pML->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapLines.push_back(pML);
                        pML->mnBALocalForKF=pKF->mnId;
                    }
        }
#endif  
    }

   
    // Fixed Keyframe: First frame previous KF to optimization window)
    list<KeyFramePtr> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for(int i=0, iend=vpNeighsKFs.size(); i<iend; i++)
    {
        if(lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFramePtr pKFi = vpNeighsKFs[i];
        if(pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
            for(vector<MapPointPtr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                MapPointPtr pMP = *vit;
                if(pMP)
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF=pKF->mnId;
                        }
            }

#if USE_LINES_LOCAL_BA_INERTIAL
            vector<MapLinePtr> vpMLs = pKFi->GetMapLineMatches();
            for(vector<MapLinePtr>::iterator vit=vpMLs.begin(), vend=vpMLs.end(); vit!=vend; vit++)
            {
                MapLinePtr pML = *vit;
                if(pML)
                    if(!pML->isBad())
                        if(pML->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapLines.push_back(pML);
                            pML->mnBALocalForKF=pKF->mnId;
                        }
            }
#endif 
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }

#if USE_LINES_LOCAL_BA_INERTIAL    
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }
#endif         

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    if(bLarge)
    {
        solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    }
    else
    {
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }


    // Set Local temporal KeyFrame vertices
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local visual KeyFrame vertices
    for(list<KeyFramePtr>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFramePtr pKFi = *it;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu) // This should be done only for keyframe just before temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);

    for(int i=0;i<N;i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            MSG_WARN_STREAM("NO INERTIAL LINK TO PREVIOUS FRAME - KF id: " << pKFi->mnId);
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if(i==N-1 || bRecInit)
            {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if(i==N-1)
                    vei[i]->setInformation(vei[i]->information()*1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
            vear[i]->setInformation(InfoA);           

            optimizer.addEdge(vear[i]);
        }
        else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (N+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);


#if USE_LINES_LOCAL_BA_INERTIAL

    // Set MapLine vertices
    const int nExpectedSizeLines = (N+lFixedKeyFrames.size())*lLocalMapLines.size();

    // Mono
    vector<EdgeLineMono*> vpEdgesMonoLines;
    vpEdgesMonoLines.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFMonoLines;
    vpEdgeKFMonoLines.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeMono;
    vpMapLineEdgeMono.reserve(nExpectedSizeLines);

    // Stereo
    vector<EdgeLineStereo*> vpEdgesStereoLines;
    vpEdgesStereoLines.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFStereoLines;
    vpEdgeKFStereoLines.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeStereo;
    vpMapLineEdgeStereo.reserve(nExpectedSizeLines);
#endif 


    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const float thHuberLineMono = sqrt(5.991);  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float chi2LineMono2 = 5.991;    
    const float thHuberLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float chi2LineStereo2 = 9.49;    
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n
    const float chi2ObjectStereo2 = 3;        

    const unsigned long iniMPid = maxKFid*5;

    map<int,int> mVisEdges;
    for(int i=0;i<N;i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for(list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        mVisEdges[(*lit)->mnId] = 0;
    }

    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if(leftIndex != -1)// Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncertainty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }


#if USE_LINES_LOCAL_BA_INERTIAL

    const unsigned long maxPointId = iniMPid+1+MapPoint::GetCurrentMaxId(); 

    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;
        if (!pML)
            continue;

        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);             
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());        
        unsigned long id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine);

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));

        const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();

        // Create visual constraints
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if (!pKFi)
                continue;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                VertexPose* VP = dynamic_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
                if(VP == NULL)
                    continue;    
                const ImuCamPose& imuCamPose = VP->estimate();

                const int leftIndex = get<0>(mit->second);
                float uRightLineStart = -1;
                float uRightLineEnd = -1;
                if(leftIndex != -1 && !pKFi->mvuRightLineStart.empty()) 
                {
                    uRightLineStart = pKFi->mvuRightLineStart[leftIndex];
                    uRightLineEnd = pKFi->mvuRightLineEnd[leftIndex];
                }

                if( leftIndex != -1)
                {
                    const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[leftIndex];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    
                    Eigen::Matrix<double,3,1> obs;
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);  

                    // Monocular observation
        #if USE_LINES_STEREO_INERTIAL                
                    if( uRightLineStart<0 || uRightLineEnd<0 )
        #endif                     
                    {
                        mVisEdges[pKFi->mnId] += Tracking::sknLineTrackWeigth;

                        EdgeLineMono* e = new EdgeLineMono(0);

                        e->setVertex(0, vertexLine);
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        
                        // Add here uncertainty
                        const float unc2 = 1.0;// pKFi->mpCamera->uncertainty2(obs);

        #if !USE_NEW_LINE_INFORMATION_MAT   
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
        #else
                        const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave]*unc2;

                        Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                        Eigen::Vector2d projMapP, projMapQ;
                        e->getMapLineProjections(projMapP, projMapQ);
                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                        e->setInformation(Info);
        #endif       

            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineMono);
            #endif    
                        e->setRobustKernel(rk);
                        
                        optimizer.addEdge(e);
                        
                        vpEdgesMonoLines.push_back(e);
                        vpEdgeKFMonoLines.push_back(pKFi);
                        vpMapLineEdgeMono.push_back(pML);
                    }
                    // Stereo-observation
                    else 
                    {
                        mVisEdges[pKFi->mnId] += Tracking::sknLineTrackWeigth;

                        EdgeLineStereo* e = new EdgeLineStereo(0);

                        e->setVertex(0, vertexLine);
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // the following two are actually derived/indirect observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                        const Eigen::Vector3f XSc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.startPointX,klUn.startPointY),pKFi->mvDepthLineStart[leftIndex] );
                        const Eigen::Vector3f XEc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.endPointX,klUn.endPointY),pKFi->mvDepthLineEnd[leftIndex] );
                        //std::cout << "XSc_backproj: " << XSc_backproj.transpose() << ", XEc_backproj: " << XEc_backproj.transpose() << std::endl; 
                        e->setBackprojections(XSc_backproj, XEc_backproj);
                        e->muWeigth = Optimizer::skMuWeightForLine3dDist;
                        
                        e->init(); // here we check the match between Bp and P (Bq and Q)

                        // Add here uncertainty
                        const float unc2 = 1.0; //pKFi->mpCamera->uncertainty2(obs.head(2));

        #if !USE_NEW_LINE_INFORMATION_MAT                   
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                        const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                        Info(0,0)*=invSigma2;
                        Info(1,1)*=invSigma2;
                        Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                        Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
        #else
                        const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave]*unc2;
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                        Eigen::Vector2d projMapP, projMapQ;
                        Eigen::Vector3d mapP, mapQ;
                        e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                        Eigen::Vector3d &backprojP = e->XSbc;
                        Eigen::Vector3d &backprojQ = e->XEbc; 

                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
            #if USE_NEW_LINE_INFORMATION_MAT_STEREO                
                        Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                        sigma2, klUn.octave,
                                        pKFi->fx, pKFi->fy, pKFi->mbfInv, 
                                        projMapP, projMapQ, 
                                        mapP, mapQ,
                                        backprojP, backprojQ);    
            #else
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                        const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                        Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                        Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
            #endif
                        
        #endif
                        
                        e->setInformation(Info);                    

                #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
                #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineStereo);
                #endif    
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);
                        
                        vpEdgesStereoLines.push_back(e);
                        vpEdgeKFStereoLines.push_back(pKFi);
                        vpMapLineEdgeStereo.push_back(pML);
                    }
                } // if( leftIndex != -1)

    #if USE_LINES_RIGHT_PROJECTION
                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 )
                    {
                        rightIndex -= pKFi->NlinesLeft;                    
                        mVisEdges[pKFi->mnId] += Tracking::sknLineTrackWeigth;

                        const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesRightUn[rightIndex];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                        
                        Eigen::Matrix<double,3,1> obs;
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);  

                        EdgeLineMono* e = new EdgeLineMono(1); // 1 = right camera index 

                        e->setVertex(0, vertexLine);
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        
                        // Add here uncertainty
                        const float unc2 = 1.0;// pKFi->mpCamera->uncertainty2(obs);
  
                        const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);   

            #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(thHuberLineMono);
            #endif    
                        e->setRobustKernel(rk);
                        
                        optimizer.addEdge(e);
                        
                        vpEdgesMonoLines.push_back(e);
                        vpEdgeKFMonoLines.push_back(pKFi);
                        vpMapLineEdgeMono.push_back(pML);                        
                    }
                } 
    #endif // USE_LINES_RIGHT_PROJECTION
            }
        }
    }

#endif  // USE_LINES_LOCAL_BA_INERTIAL    

    //cout << "Total map points: " << lLocalMapPoints.size() << endl;
    for(map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    {
#if 0        
        MSG_ASSERT(mit->second >=3, "number of visual edge observation must be >=3! current observations num: " << mit->second);
        //assert(mit->second>=3);
#else  
        //NOTE: [Luigi] added this instead of the assert! If the KF is not well constrained we may want to consider it fixed!
        //      See also this open issue: 
        //      https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/145
        if(mit->second<3)
        {
            std::cout << IoColor::Yellow() << "LocalInertialBA - KF " << mit->first << " with only " << mit->second << " observations" << IoColor::Default() << std::endl;               
    #if 0 
            VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(mit->first));           
            VP->setFixed(true);
            std::cout << IoColor::Yellow() << "\t Adding fixed KF " << mit->first << IoColor::Default() << std::endl;     
    #endif             
        }
#endif         
    }     

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it); // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    vector<pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth<10.f;

        if(pMP->isBad())
            continue;

        if((e->chi2()>chi2Mono2 && !bClose) || (e->chi2()>1.5f*chi2Mono2 && bClose) || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


#if USE_LINES_LOCAL_BA_INERTIAL
	vector<pair<KeyFramePtr,MapLinePtr> > vToEraseLines;
    vToEraseLines.reserve(vpEdgesMonoLines.size()+vpEdgesStereoLines.size());

    // Check line inlier observations
    // Mono Lines
    for(size_t i=0, iend=vpEdgesMonoLines.size(); i<iend;i++)
    {
        EdgeLineMono* e = vpEdgesMonoLines[i];
        MapLinePtr pML = vpMapLineEdgeMono[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>chi2LineMono2 || !e->areDepthsPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMonoLines[i];
            vToEraseLines.push_back(make_pair(pKFi,pML));
        }
    }

    // Stereo Lines
    for(size_t i=0, iend=vpEdgesStereoLines.size(); i<iend;i++)
    {
        EdgeLineStereo* e = vpEdgesStereoLines[i];
        MapLinePtr pML = vpMapLineEdgeStereo[i];

        if(pML->isBad())
            continue;

        if(e->zeroMuChi2()>chi2LineStereo2 || !e->areDepthsPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereoLines[i];
            vToEraseLines.push_back(make_pair(pKFi,pML));
        }
    }
#endif     

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);


    // TODO: Some convergence problems have been detected here
    if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }


    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

#if USE_LINES_LOCAL_BA_INERTIAL
    // Erase line outliers
    if(!vToEraseLines.empty())
    {
        for(size_t i=0;i<vToEraseLines.size();i++)
        {
            KeyFramePtr pKFi = vToEraseLines[i].first;
            MapLinePtr pMLi = vToEraseLines[i].second;
            pKFi->EraseMapLineMatch(pMLi);
            pMLi->EraseObservation(pKFi);
        }
    }
#endif  // USE_LINES_LOCAL_BA_INERTIAL    


    for(list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;
        pKFi->mnLBACount++;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));

        }
    }

    // Local visual KeyFrame
    for(list<KeyFramePtr>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFramePtr pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;
        pKFi->mnLBACount++;
    }

    //Points
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

#if USE_LINES_LOCAL_BA_INERTIAL
    //Lines
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;
        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pML->mnId+maxPointId+1));
        const Eigen::Vector3f pStartNew = (static_cast<const Eigen::Vector3d >(vLine->estimate().head(3))).cast<float>();
        const Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Vector3d >(vLine->estimate().tail(3))).cast<float>();                
        pML->SetWorldEndPoints(pStartNew, pEndNew);
        pML->UpdateNormalAndDepth();

        if(vLine->isBad()) pML->SetBadFlag();
    }
#endif   

    pMap->IncreaseChangeIndex();
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
{
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end-start+1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end+1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(),H.cols());
    if(a>0)
    {
        Hn.block(0,0,a,a) = H.block(0,0,a,a);
        Hn.block(0,a+c,a,b) = H.block(0,a,a,b);
        Hn.block(a+c,0,b,a) = H.block(a,0,b,a);
    }
    if(a>0 && c>0)
    {
        Hn.block(0,a,a,c) = H.block(0,a+b,a,c);
        Hn.block(a,0,c,a) = H.block(a+b,0,c,a);
    }
    if(c>0)
    {
        Hn.block(a,a,c,c) = H.block(a+b,a+b,c,c);
        Hn.block(a,a+c,c,b) = H.block(a+b,a,c,b);
        Hn.block(a+c,a,b,c) = H.block(a,a+b,b,c);
    }
    Hn.block(a+c,a+c,b,b) = H.block(a,a,b,b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a+c,a+c,b,b),Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv=svd.singularValues();
    for (int i=0; i<b; ++i)
    {
        if (singularValues_inv(i)>1e-6)
            singularValues_inv(i)=1.0/singularValues_inv(i);
        else singularValues_inv(i)=0;
    }
    Eigen::MatrixXd invHb = svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
    Hn.block(0,0,a+c,a+c) = Hn.block(0,0,a+c,a+c) - Hn.block(0,a+c,a+c,b)*invHb*Hn.block(a+c,0,b,a+c);
    Hn.block(a+c,a+c,b,b) = Eigen::MatrixXd::Zero(b,b);
    Hn.block(0,a+c,a+c,b) = Eigen::MatrixXd::Zero(a+c,b);
    Hn.block(a+c,0,b,a+c) = Eigen::MatrixXd::Zero(b,a+c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(),H.cols());
    if(a>0)
    {
        res.block(0,0,a,a) = Hn.block(0,0,a,a);
        res.block(0,a,a,b) = Hn.block(0,a+c,a,b);
        res.block(a,0,b,a) = Hn.block(a+c,0,b,a);
    }
    if(a>0 && c>0)
    {
        res.block(0,a+b,a,c) = Hn.block(0,a,a,c);
        res.block(a+b,0,c,a) = Hn.block(a,0,c,a);
    }
    if(c>0)
    {
        res.block(a+b,a+b,c,c) = Hn.block(a,a,c,c);
        res.block(a+b,a,c,b) = Hn.block(a,a+c,c,b);
        res.block(a,a+b,b,c) = Hn.block(a+c,a,b,c);
    }

    res.block(a,a,b,b) = Hn.block(a+c,a+c,b,b);

    return res;
}

Eigen::MatrixXd Optimizer::Condition(const Eigen::MatrixXd &H, const int &start, const int &end)
{
    // Size of block before block to condition
    const int a = start;
    // Size of block to condition
    const int b = end+1-start;

    // Set to zero elements related to block b(start:end,start:end)
    // a  | ab | ac       a  | 0 | ac
    // ba | b  | bc  -->  0  | 0 | 0
    // ca | cb | c        ca | 0 | c

    Eigen::MatrixXd Hn = H;

    Hn.block(a,0,b,H.cols()) = Eigen::MatrixXd::Zero(b,H.cols());
    Hn.block(0,a,H.rows(),b) = Eigen::MatrixXd::Zero(H.rows(),b);

    return Hn;
}

Eigen::MatrixXd Optimizer::Sparsify(const Eigen::MatrixXd &H, const int &start1, const int &end1, const int &start2, const int &end2)
{
    // Goal: remove link between a and b
    // p(a,b,c) ~ p(a,b,c)*p(a|c)/p(a|b,c) => H' = H + H1 - H2
    // H1: marginalize b and condition c
    // H2: condition b and c
    Eigen::MatrixXd Hac = Marginalize(H,start2,end2);
    Eigen::MatrixXd Hbc = Marginalize(H,start1,end1);
    Eigen::MatrixXd Hc = Marginalize(Hac,start1,end1);

    return Hac+Hbc-Hc;
}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel, bool bGauss, float priorG, float priorA)
{
    Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    if (priorG!=0.f)
        solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        if (bFixedVel)
            VV->setFixed(true);
        else
            VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    if (bFixedVel)
        VG->setFixed(true);
    else
        VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    if (bFixedVel)
        VA->setFixed(true);
    else
        VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(maxKFid*2+5);
    VS->setFixed(!bMono); // Fixed for stereo case
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFramePtr,KeyFramePtr> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());
    //std::cout << "build optimization graph" << std::endl;

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(!pKFi->mpImuPreintegrated)
                std::cout << "Not preintegrated measurement" << std::endl;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    scale = VS->estimate();

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();
    scale = VS->estimate();


    IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
    Rwg = VGDir->estimate().Rwg;

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);


    }
}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
{
    int its = 200; // Check number of iterations
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    solver->setUserLambdaInit(1e3);
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(1.0);
    VS->setId(maxKFid*2+5);
    VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFramePtr,KeyFramePtr> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();

    IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate();
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);
    }
}

void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
{
    int its = 10;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr); 
#endif // USE_G2O_NEW

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (all variables are fixed)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+1+(pKFi->mnId));
        VV->setFixed(true);
        optimizer.addVertex(VV);

        // Vertex of fixed biases
        VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
        VG->setId(2*(maxKFid+1)+(pKFi->mnId));
        VG->setFixed(true);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(vpKFs.front());
        VA->setId(3*(maxKFid+1)+(pKFi->mnId));
        VA->setFixed(true);
        optimizer.addVertex(VA);
    }

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(4*(maxKFid+1));
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(4*(maxKFid+1)+1);
    VS->setFixed(false);
    optimizer.addVertex(VS);

    // Graph edges
    int count_edges = 0;
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
                
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex((maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex((maxKFid+1)+pKFi->mnId);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(2*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(3*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4*(maxKFid+1));
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(4*(maxKFid+1)+1);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) +  ", " + to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

                continue;
            }
            count_edges++;
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ei->setRobustKernel(rk);
            rk->setDelta(1.f);
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(its);
    optimizer.computeActiveErrors();
    float err_end = optimizer.activeRobustChi2();
    // Recover optimized data
    scale = VS->estimate();
    Rwg = VGDir->estimate().Rwg;
}

// Local BA in welding area when two maps are merged (Ok lines, OK objects)
void Optimizer::LocalBundleAdjustment(KeyFramePtr pMainKF, vector<KeyFramePtr> vpAdjustKF, vector<KeyFramePtr> vpFixedKF, bool *pbStopFlag, bool useLinesOrObject)
{
    bool bShowImages = false;

    vector<MapPointPtr> vpMPs;
    vector<MapLinePtr> vpMLs;    
    vector<MapObjectPtr> vpMOs;        

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver;
    
#if USE_LINES_LOCAL_BA || USE_OBJECTS_LOCAL_BA 

#if USE_BA_VARIABLE_SIZE_SOLVER        
    if(useLinesOrObject)
    {
        
#ifdef USE_G2O_NEW        
        solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else  // USE_G2O_NEW       
        g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW  
        
    }
    else
#endif // USE_BA_VARIABLE_SIZE_SOLVER     
    {
        
#ifdef USE_G2O_NEW 
        solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>()));        
#else  // USE_G2O_NEW                
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);        
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW    
        
    }
#else // USE_LINES_LOCAL_BA
    
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);    
    
#endif // USE_LINES_LOCAL_BA    
    
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    /*if(numObjects>0)
    {
        solver->setUserLambdaInit(1e-16);           
    }*/

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    set<KeyFramePtr> spKeyFrameBA;

    Map* pCurrentMap = pMainKF->GetMap();

    // Set fixed KeyFrame vertices
    int numInsertedPoints = 0;
    for(KeyFramePtr pKFi : vpFixedKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPointPtr> spViewMPs = pKFi->GetMapPoints();
        for(MapPointPtr pMPi : spViewMPs)
        {
            if(pMPi)
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

                    if(pMPi->mnBALocalForMerge!=pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge=pMainKF->mnId;
                        numInsertedPoints++;
                    }
        }
        
#if USE_LINES_LOCAL_BA   
        set<MapLinePtr> spViewMLs = pKFi->GetMapLines();
        for(MapLinePtr  pMLi : spViewMLs)
        {
            if(pMLi)
                if(!pMLi->isBad() && pMLi->GetMap() == pCurrentMap)

                    if(pMLi->mnBALocalForMerge!=pMainKF->mnId)
                    {
                        vpMLs.push_back(pMLi);
                        pMLi->mnBALocalForMerge=pMainKF->mnId;
                    }
        }        
#endif         
        
#if USE_OBJECTS_LOCAL_BA   
        set<MapObjectPtr> spViewMOs = pKFi->GetMapObjects();
        for(MapObjectPtr  pMOi : spViewMOs)
        {
            if(pMOi)
                if(!pMOi->isBad() && pMOi->GetMap() == pCurrentMap)

                    if(pMOi->mnBALocalForMerge!=pMainKF->mnId)
                    {
                        vpMOs.push_back(pMOi);
                        pMOi->mnBALocalForMerge=pMainKF->mnId;
                    }
        }        
#endif             

        spKeyFrameBA.insert(pKFi);
    }

    // Set non fixed Keyframe vertices
    set<KeyFramePtr> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0;
    for(KeyFramePtr pKFi : vpAdjustKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            continue;

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPointPtr> spViewMPs = pKFi->GetMapPoints();
        for(MapPointPtr pMPi : spViewMPs)
        {
            if(pMPi)
            {
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                {
                    if(pMPi->mnBALocalForMerge != pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge = pMainKF->mnId;
                        numInsertedPoints++;
                    }
                }
            }
        }
        
#if USE_LINES_LOCAL_BA  
        set<MapLinePtr> spViewMLs = pKFi->GetMapLines();
        for(MapLinePtr  pMLi : spViewMLs)
        {
            if(pMLi)
            {
                if(!pMLi->isBad() && pMLi->GetMap() == pCurrentMap)
                {
                    if(pMLi->mnBALocalForMerge != pMainKF->mnId)
                    {
                        vpMLs.push_back(pMLi);
                        pMLi->mnBALocalForMerge = pMainKF->mnId;
                    }
                }
            }
        }        
#endif 

#if USE_OBJECTS_LOCAL_BA
        set<MapObjectPtr> spViewMOs = pKFi->GetMapObjects();
        for(MapObjectPtr  pMOi : spViewMOs)
        {
            if(pMOi)
            {
                if(!pMOi->isBad() && pMOi->GetMap() == pCurrentMap)
                {
                    if(pMOi->mnBALocalForMerge != pMainKF->mnId)
                    {
                        vpMOs.push_back(pMOi);
                        pMOi->mnBALocalForMerge = pMainKF->mnId;
                    }
                }
            }
        }           
#endif 
        

        spKeyFrameBA.insert(pKFi);
    }

    
    unsigned long maxPointId = maxKFid+1+MapPoint::GetCurrentMaxId();     
    
    const int nExpectedSize = (vpAdjustKF.size()+vpFixedKF.size())*vpMPs.size();

    vector<PLVS2::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // point stereo 
#if !USE_RGBD_POINT_REPROJ_ERR      
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
#else
    vector<g2o::EdgeRgbdSE3ProjectXYZ*> vpEdgesStereo;    
#endif
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);
    
#if USE_LINES_LOCAL_BA         
        
    const int nExpectedSizeLines = (vpAdjustKF.size()+vpFixedKF.size())*vpMLs.size();

    // lines mono    
    vector<g2o::EdgeSE3ProjectLine*> vpEdgesLineMono;
    vpEdgesLineMono.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFLineMono; 
    vpEdgeKFLineMono.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeMono; 
    vpMapLineEdgeMono.reserve(nExpectedSizeLines);
    
    // lines stereo
    vector<g2o::EdgeSE3ProjectStereoLine*> vpEdgesLineStereo;
    vpEdgesLineStereo.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFLineStereo;    
    vpEdgeKFLineStereo.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeStereo; 
    vpMapLineEdgeStereo.reserve(nExpectedSizeLines);
    
#endif // USE_LINES_LOCAL_BA    
    
#if USE_OBJECTS_LOCAL_BA
    
    unsigned long maxLineId = maxPointId+1+MapLine::GetCurrentMaxId(); 

    const int nExpectedSizeObjects = vpMOs.size()*5; // 5 views per object on average 

    vector<g2o::EdgeSim3SE3*> vpEdgesObject;
    vpEdgesObject.reserve(nExpectedSizeObjects);
    
    vector<KeyFramePtr> vpEdgeKFObject; 
    vpEdgeKFObject.reserve(nExpectedSizeObjects);

    vector<MapObjectPtr > vpMapObjectEdge; 
    vpMapObjectEdge.reserve(nExpectedSizeObjects);    
    
    std::vector<double> vEdgesObjectSquaredErrors;
    vEdgesObjectSquaredErrors.reserve(nExpectedSizeObjects);        
     

#endif // USE_OBJECTS_LOCAL_BA        
    
    const float thHuberMono = sqrt(5.991);      // chi-squared 2 DOFS 
    const float thHuberStereo = sqrt(7.815);    // chi-squared 3 DOFS
    const float thHuberLineMono = sqrt(5.991);  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    
    // -----------------------------------------------------------------------------
    

    // Set MapPoint vertices
    map<KeyFramePtr, int> mpObsKFs;
    map<KeyFramePtr, int> mpObsFinalKFs;
    map<MapPointPtr, int> mpObsMPs;
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPointPtr pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
        const int id = pMPi->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        g2o::OptimizableGraph::Vertex* vertexPoint = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,tuple<int,int>> observations = pMPi->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            const int leftIndex = get<0>(mit->second);
            if(pKF->isBad() || leftIndex == -1 || pKF->mnId>maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(leftIndex))
                continue;

            g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));
            if(vertexKFi == NULL)
                    continue;
                            
            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

            if(pKF->mvuRight[leftIndex]<0) //Monocular
            {
                mpObsMPs[pMPi]++;
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                PLVS2::EdgeSE3ProjectXYZ* e = new PLVS2::EdgeSE3ProjectXYZ();

                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexPoint);
                e->setVertex(1, vertexKFi);                
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberMono);

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsMPs[pMPi]+=2;
                Eigen::Matrix<double,3,1> obs;
#if !USE_RGBD_POINT_REPROJ_ERR                 
                const float kp_ur = pKF->mvuRight[leftIndex];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
#else
                const float kpDelta = pKFi->mvDepth[leftIndex];
                obs << kpUn.pt.x, kpUn.pt.y, kpDelta;

                g2o::EdgeRgbdSE3ProjectXYZ* e = new g2o::EdgeRgbdSE3ProjectXYZ();                    
#endif
                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexPoint);
                e->setVertex(1, vertexKFi);                
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                
#if !USE_RGBD_POINT_REPROJ_ERR                      
    #if !USE_NEW_STEREO_POINT_INFORMATION_MAT              
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;                
    #else
                Eigen::Matrix3d Info = Eigen::Matrix3d::Zero();                     
                SetStereoPointInformationMat(Info, invSigma2, pKFi->mbf, pKFi->mbfInv, pKFi->mvDepth[mit->second]);
    #endif       
#else
                Eigen::Matrix3d Info = Eigen::Matrix3d::Zero();                     
                SetRgbdPointInformationMat(Info, invSigma2, pKFi->mbfInv, kpDelta);
#endif
                
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberStereo);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
#if !USE_RGBD_POINT_REPROJ_ERR                      
                e->bf = pKF->mbf;
#endif

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
        }
    }

#if USE_LINES_LOCAL_BA    // ---------------------------------------------------

    // Set MapLine vertices
    map<KeyFramePtr, int> mpLineObsKFs;
    map<KeyFramePtr, int> mpLineObsFinalKFs;
    map<MapLinePtr , int> mpObsMLs;
    for(unsigned int i=0; i < vpMLs.size(); ++i)
    {
        MapLinePtr  pMLi = vpMLs[i];
        if(pMLi->isBad())
            continue;
        
        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pMLi->GetWorldEndPoints(posStart, posEnd);                       
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pMLi->GetLength());        
        // vLine->P = posStart.cast<double>();
        // vLine->Q = posEnd.cast<double>();
        int id = pMLi->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine); 

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,tuple<int,int>> observations = pMLi->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            const int leftIndex = get<0>(mit->second);
            if(pKF->isBad() || leftIndex == -1 || pKF->mnId>maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapLine(leftIndex))
                continue;

            g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));
            if(vertexKFi == NULL)
                    continue;
                            
            nEdges++;

            const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesUn[leftIndex];
            Line2DRepresentation lineRepresentation;
            Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);

#if USE_LINE_STEREO                
            if( (!pKF->mvuRightLineStart.empty()) && (pKF->mvuRightLineStart[leftIndex]<0) || (pKF->mvuRightLineEnd[leftIndex]<0) ) //Monocular
#endif                
            {
                mpObsMLs[pMLi]++;
                
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                g2o::EdgeSE3ProjectLine* e = new g2o::EdgeSE3ProjectLine();                

                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexLine);
                e->setVertex(1, vertexKFi);                
                e->setMeasurement(obs);
                
                //e->pCamera = pKF->mpCamera; /// < TODO: Luigi add camera jac management here with mpCamera!

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;    
                    
#if !USE_NEW_LINE_INFORMATION_MAT   
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
#else
                const float sigma2 = pKF->mvLineLevelSigma2[klUn.octave];

                Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                Eigen::Vector2d projMapP, projMapQ;
                e->getMapLineProjections(projMapP, projMapQ);
                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
                e->setInformation(Info);
#endif           

        #if USE_CAUCHY_KERNEL_FOR_LINES
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberLineMono);
        #endif   
                e->setRobustKernel(rk);

                optimizer.addEdge(e);

                vpEdgesLineMono.push_back(e);
                vpEdgeKFLineMono.push_back(pKF);
                vpMapLineEdgeMono.push_back(pMLi);

                mpLineObsKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsMLs[pMLi]+=2;
                
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);   

                g2o::EdgeSE3ProjectStereoLine* e = new g2o::EdgeSE3ProjectStereoLine();
                
                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexLine);
                e->setVertex(1, vertexKFi);                   
                e->setMeasurement(obs);
                
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                // the following two are actually derived observations (using also depth measurements) but we keep them cached inside the edge for simplicity 
                e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pKF->mvDepthLineStart[leftIndex]);
                e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pKF->mvDepthLineEnd[leftIndex]);

                e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 
                e->mu = Optimizer::skMuWeightForLine3dDist;

                e->init();

#if !USE_NEW_LINE_INFORMATION_MAT                   
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                    
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;                    
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                Info(0,0)*=invSigma2;
                Info(1,1)*=invSigma2;
                Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;            
#else
                const float sigma2 = pKF->mvLineLevelSigma2[klUn.octave];
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                Eigen::Vector2d projMapP, projMapQ;
                Eigen::Vector3d mapP, mapQ;
                e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                Eigen::Vector3d &backprojP = e->XSbc;
                Eigen::Vector3d &backprojQ = e->XEbc; 

                Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                           klUn.startPointX,klUn.startPointY, 
                           klUn.endPointX,klUn.endPointY, 
                           lineRepresentation.nx, lineRepresentation.ny, 
                           projMapP, projMapQ);
#if USE_NEW_LINE_INFORMATION_MAT_STEREO  
                Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                sigma2, klUn.octave, 
                                pKF->fx, pKF->fy, pKF->mbfInv, 
                                projMapP, projMapQ, 
                                mapP, mapQ,
                                backprojP, backprojQ);   
#else
                const float invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                      
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;                    
                Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
#endif

#endif

                e->setInformation(Info);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                rk->setDelta(thHuberLineStereo);
        #endif   
                e->setRobustKernel(rk);

                optimizer.addEdge(e);
                vpEdgesLineStereo.push_back(e);
                vpEdgeKFLineStereo.push_back(pKF);
                vpMapLineEdgeStereo.push_back(pMLi);

                mpLineObsKFs[pKF]++;
            }
        }
    }
    
#endif     // USE_LINES_LOCAL_BA  
    
#if USE_OBJECTS_LOCAL_BA  // ---------------------------------------------------   
    
    // Set MapObject vertices
    map<KeyFramePtr, int> mpObjectObsKFs;
    map<KeyFramePtr, int> mpObjectObsFinalKFs;
    map<MapObjectPtr , int> mpObsMOs;
    
    bool bFixScale = false;
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    for(unsigned int i=0; i < vpMOs.size(); ++i)
    {
        MapObjectPtr pMObj = vpMOs[i];
        if(pMObj->isBad())
            continue;

        g2o::VertexSim3Expmap* vObject = new g2o::VertexSim3Expmap();
        const Eigen::Matrix<double,3,3> Row = pMObj->GetRotation().cast<double>();
        const Eigen::Matrix<double,3,1> tow = pMObj->GetTranslation().cast<double>();
        const double objectScale = pMObj->GetScale();
        g2o::Sim3 Sow(Row,tow,1./objectScale); // Sow = [Row/s, tow; 0, 1]  
        //std::cout << "LBA - Sow: " << Converter::toCvMat(Sow) << std::endl; 
        vObject->setEstimate(Sow);
        int id = pMObj->mnId+maxLineId+1;
        vObject->setId(id);
        vObject->setMarginalized(true);
        vObject->_fix_scale = bFixScale;        
        optimizer.addVertex(vObject);

        g2o::OptimizableGraph::Vertex* vertexObject = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,ObjectObservation> observations = pMObj->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,ObjectObservation>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId) // || !pKF->GetMapObjects(mit->second))
                continue;

            g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));
            if(vertexKFi == NULL)
                    continue;
                            
            nEdges++;

            const ObjectObservation& observation = mit->second;

            //if(!observation.bFromKF) continue; 

            mpObsMOs[pMObj]++;
            
            const Sophus::SE3f Tko = observation.GetSE3(); // from object to keyframe               
            const Eigen::Matrix<double,3,3> Rko = Tko.rotationMatrix().cast<double>();
            const Eigen::Matrix<double,3,1> tko = Tko.translation().cast<double>();                
            const double observedScale = observation.fScale;

            const g2o::Sim3 Sko(Rko,tko,observedScale); // Sko = [s*Rko, tko; 0, 1]             

            g2o::EdgeSim3SE3* e = new g2o::EdgeSim3SE3();
            e->setVertex(0, vertexObject);  // Sim3   Sow              
            e->setVertex(1, vertexKFi); // SE3        Tkw                   
            e->setMeasurement(Sko);
            e->setInformation(matLambda);
            optimizer.addEdge(e);

        #if USE_CAUCHY_KERNEL_FOR_OBJECTS
            g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(thHuberObjectTimesSigma);
        #endif 

            e->setRobustKernel(rk);                               

            vpEdgesObject.push_back(e);
            vpMapObjectEdge.push_back(pMObj);                
            vpEdgeKFObject.push_back(pKF);

            e->computeError();
            vEdgesObjectSquaredErrors.push_back(e->chi2()); // here we accumulate the squared errors (given that matLambda is the identity)
                
#if VERBOSE_LOCAL_BA                   
            std::cout << "chi2: " << vEdgesObjectSquaredErrors[vEdgesObjectSquaredErrors.size()-1] << std::endl;        
#endif     

            mpObjectObsKFs[pKF]++;
        }
    }
    
    // we need this step in order to set a decent information matrix 
    if(vEdgesObjectSquaredErrors.size()>0)
    {
        double sigmaEdgesObjectsSquared = Utils::FindSigmaSquared(vEdgesObjectSquaredErrors); // robust estimation of variance   
        double invSigmaEdgesObjectsSquared = 1./sigmaEdgesObjectsSquared;
        for(size_t i=0, iend=vEdgesObjectSquaredErrors.size(); i<iend;i++)
        {
            vpEdgesObject[i]->information() *= invSigmaEdgesObjectsSquared;  // N.B.: in this way one has e->chi2() = err_k^2/sigma_err^2
        }
    }    
    
#endif // USE_OBJECTS_LOCAL_BA  ------------------------------------------------       

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    map<unsigned long int, int> mWrongObsKF;
    map<unsigned long int, int> mWrongLineObsKF;    
    map<unsigned long int, int> mWrongObjectObsKF;       
    
    if(bDoMore)
    {
        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            PLVS2::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPointPtr pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1); /// < NOTE: outliers are removed here! 
                badMonoMP++;
            }
            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
#if !USE_RGBD_POINT_REPROJ_ERR         
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
#else
            g2o::EdgeRgbdSE3ProjectXYZ* e = vpEdgesStereo[i];
#endif
            MapPointPtr pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1); /// < NOTE: outliers are removed here! 
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        Verbose::PrintMess("[LBA]: First optimization(Huber), there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " stereo bad edges", Verbose::VERBOSITY_DEBUG);


#if USE_LINES_LOCAL_BA    
        int nLineMonoBadObs = 0;
        for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectLine* e = vpEdgesLineMono[i];
            MapLinePtr pML = vpMapLineEdgeMono[i];

            if(pML->isBad())
                continue;

            if(e->chi2()>5.991 || !e->areDepthsPositive())
            {
                e->setLevel(1);   /// < NOTE: outliers are removed here! 
                nLineMonoBadObs++;
            }

            e->setRobustKernel(0); 
        }

    #if USE_LINE_STEREO       
        int nLineStereoBadObs = 0;
        for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectStereoLine* e = vpEdgesLineStereo[i];
            MapLinePtr pML = vpMapLineEdgeStereo[i];

            if(pML->isBad())
                continue;

            //if(e->chi2()>9.49 || !e->areDepthsPositive()) 
            // N.B.: outliers are identified by just considering how close is the representative 3D line to the back-projected measured 2D line  
            //       without considering (mu=0) how much the representative 3D points Q,P are close to the back-projected Qi,Pi from the measured 2D points pi,qi           
            if(e->zeroMuChi2()>9.49 || !e->areDepthsPositive())  
            {
                e->setLevel(1);  /// < NOTE: outliers are removed here!
                nLineStereoBadObs++;  
            }

            e->setRobustKernel(0);  
        }
    #endif

#endif // USE_LINES_LOCAL_BA


#if USE_OBJECTS_LOCAL_BA    
        int nObjectBadObs = 0;
        for(size_t i=0, iend=vpEdgesObject.size(); i<iend;i++)
        {
            g2o::EdgeSim3SE3* e = vpEdgesObject[i];
            MapObjectPtr pMObj = vpMapObjectEdge[i];

            if(pMObj->isBad())
                continue;

            if(e->chi2()>3) // err_k^2/sigma_err^2 > 3
            {
                e->setLevel(1);   /// < NOTE: outliers are removed here! 
                nObjectBadObs++;
            }

            e->setRobustKernel(0);  
        }
#endif            
        
        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
    set<MapPointPtr> spErasedMPs;
    set<KeyFramePtr> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        PLVS2::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

#if USE_LINES_LOCAL_BA
    
    vector<pair<KeyFramePtr,MapLinePtr> > vLineToErase;
    vLineToErase.reserve(vpEdgesLineMono.size()+vpEdgesLineStereo.size());
    set<MapLinePtr> spErasedMLs;
    set<KeyFramePtr> spErasedLineKFs;

    // Check inlier observations
    int badMonoML = 0, badStereoML = 0;
    for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectLine* e = vpEdgesLineMono[i];
        MapLinePtr pML = vpMapLineEdgeMono[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>5.991 || !e->areDepthsPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFLineMono[i];
            vLineToErase.push_back(make_pair(pKFi,pML));
            mWrongLineObsKF[pKFi->mnId]++;
            badMonoML++;

            spErasedMLs.insert(pML);
            spErasedLineKFs.insert(pKFi);
        }
    }

#if USE_LINE_STEREO  

#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
    int num3DObservartionInliers = 0; 
    double totSquaredAlignmentError = 0; 
    double totSquaredEndPointsDeviations = 0; 
#endif
    
    for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectStereoLine* e = vpEdgesLineStereo[i];
        MapLinePtr pML = vpMapLineEdgeStereo[i];

        if(pML->isBad())
            continue;

        //if(e->chi2()>9.49 || !e->areDepthsPositive())     
        // N.B.: outliers are identified by just considering how close is the representative 3D line to the back-projected measured 2D line  
        //       without considering (mu=0) how much the representative 3D points Q,P are close to the back-projected Qi,Pi from the measured 2D points pi,qi        
        if(e->zeroMuChi2()>9.49 || !e->areDepthsPositive())    
        {
            KeyFramePtr pKFi = vpEdgeKFLineStereo[i];
            vLineToErase.push_back(make_pair(pKFi,pML));
            mWrongLineObsKF[pKFi->mnId]++;
            badStereoML++;

            spErasedMLs.insert(pML);
            spErasedLineKFs.insert(pKFi);
        }
        else
        {
            //pML->muNumLineBAFailures = 0; 
            
#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
            num3DObservartionInliers++; 
            totSquaredAlignmentError += e->computeSquared3DError(); 
            totSquaredEndPointsDeviations += e->computeSquaredEndPointsDeviations();
#endif            
        }
    }  
    
#if VERBOSE_TOT_3DLINE_ALIGNMENT_ERROR    
    const int TwoNum3DObservartionInliers = 2*num3DObservartionInliers;
    std::cout << "Local BA - average 3D line error: " << sqrt( totSquaredAlignmentError/TwoNum3DObservartionInliers ) 
              << ", average endpoints deviation: " << sqrt( totSquaredEndPointsDeviations/TwoNum3DObservartionInliers ) 
              << " , num 3D obs inliers: " << num3DObservartionInliers <<  std::endl; 
#endif       
    
#endif // USE_LINE_STEREO
    
#endif // USE_LINES_LOCAL_BA      
    
    
#if USE_OBJECTS_LOCAL_BA
    vector<pair<KeyFramePtr,MapObjectPtr> > vObjectToErase;
    vObjectToErase.reserve(vpEdgesObject.size());
    set<MapObjectPtr> spErasedMOs;
    set<KeyFramePtr> spErasedObjectKFs;

    // Check inlier observations
    int badMonoMO = 0;
    for(size_t i=0, iend=vpEdgesObject.size(); i<iend;i++)
    {
        g2o::EdgeSim3SE3* e = vpEdgesObject[i];
        MapObjectPtr pMO = vpMapObjectEdge[i];

        if(pMO->isBad())
            continue;

        if(e->chi2()>3)  // err_k^2/sigma_err^2 > 3
        {
            KeyFramePtr pKFi = vpEdgeKFObject[i];
            vObjectToErase.push_back(make_pair(pKFi,pMO));
            mWrongObjectObsKF[pKFi->mnId]++;
            badMonoMO++;

            spErasedMOs.insert(pMO);
            spErasedObjectKFs.insert(pKFi);
        }
    }    
    
#endif // USE_OBJECTS_LOCAL_BA
    
    Verbose::PrintMess("LBA: Second optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

    // Get Map Mutex
    unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPointPtr pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        const map<KeyFramePtr,tuple<int,int>> observations = pMPi->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            const int leftIndex = get<0>(mit->second);
            if(pKF->isBad() || leftIndex == -1 || pKF->mnId>maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(leftIndex))
                continue;

            if(pKF->mvuRight[leftIndex]<0) //Monocular
            {
                mpObsFinalKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsFinalKFs[pKF]++;
            }
        }
    }
    
    
#if USE_LINES_LOCAL_BA

    if(!vLineToErase.empty())
    {
        map<KeyFramePtr, int> mpMLs_in_KF;
        for(KeyFramePtr pKFi : spErasedLineKFs)
        {
            int num_MLs = pKFi->GetMapLines().size();
            mpMLs_in_KF[pKFi] = num_MLs;
        }

        Verbose::PrintMess("LBA: There are " + to_string(vLineToErase.size()) + " line observations whose will be deleted from the map", Verbose::VERBOSITY_DEBUG);
        for(size_t i=0;i<vLineToErase.size();i++)
        {
            KeyFramePtr pKFi = vLineToErase[i].first;
            MapLinePtr pMLi = vLineToErase[i].second;
            pKFi->EraseMapLineMatch(pMLi);
            pMLi->EraseObservation(pKFi);
        }

        Verbose::PrintMess("LBA: " + to_string(spErasedMLs.size()) + " MLs had deleted line observations", Verbose::VERBOSITY_DEBUG);
        Verbose::PrintMess("LBA: Current map is " + to_string(pMainKF->GetMap()->GetId()), Verbose::VERBOSITY_DEBUG);
        int numErasedML = 0;
        for(MapLinePtr  pMLi : spErasedMLs)
        {
            if(pMLi->isBad())
            {
                Verbose::PrintMess("LBA: ML " + to_string(pMLi->mnId) + " has lost almost all the observations, its origin map is " + to_string(pMLi->mnOriginMapId), Verbose::VERBOSITY_DEBUG);
                numErasedML++;
            }
        }
        Verbose::PrintMess("LBA: " + to_string(numErasedML) + " MLs had deleted from the map", Verbose::VERBOSITY_DEBUG);

        for(KeyFramePtr pKFi : spErasedLineKFs)
        {
            int num_MLs = pKFi->GetMapLines().size();
            int num_init_MLs = mpMLs_in_KF[pKFi];
            Verbose::PrintMess("LBA: Initially KF " + to_string(pKFi->mnId) + " had " + to_string(num_init_MLs) + ", at the end has " + to_string(num_MLs), Verbose::VERBOSITY_DEBUG);
        }
    }
    for(unsigned int i=0; i < vpMLs.size(); ++i)
    {
        MapLinePtr  pMLi = vpMLs[i];
        if(pMLi->isBad())
            continue;

        const map<KeyFramePtr,tuple<int,int>> observations = pMLi->GetObservations();
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            const int leftIndex = get<0>(mit->second);
            if(pKF->isBad() || leftIndex == -1 || pKF->mnId>maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapLine(leftIndex))
                continue;

            const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesUn[leftIndex];

            // if( (pKF->mvuRightLineStart[leftIndex]<0) || (pKF->mvuRightLineEnd[leftIndex]<0) ) //Monocular    
            // {
            //     mpLineObsFinalKFs[pKF]++;
            // }
            // else // RGBD or Stereo
            {
                mpLineObsFinalKFs[pKF]++;
            }
        }
    }
    
#endif // USE_LINES_LOCAL_BA
    
#if USE_OBJECTS_LOCAL_BA
    if(!vObjectToErase.empty())
    {
        for(size_t i=0;i<vObjectToErase.size();i++)
        {
            KeyFramePtr pKFi = vObjectToErase[i].first;
            MapObjectPtr pMOi = vObjectToErase[i].second;
            pKFi->EraseMapObjectMatch(pMOi);
            pMOi->EraseObservation(pKFi);
        }
    }    
#endif            

    
    // Recover optimized data
    // Keyframes
    for(KeyFramePtr pKFi : vpAdjustKF)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());

#if 1
        // collect some statistics for debugging 
        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPointPtr> vpMonoMPsOpt, vpStereoMPsOpt;
        vector<MapPointPtr> vpMonoMPsBad, vpStereoMPsBad;

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            PLVS2::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPointPtr pMP = vpMapPointEdgeMono[i];
            KeyFramePtr pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);

            }
            else
            {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPointPtr pMP = vpMapPointEdgeStereo[i];
            KeyFramePtr pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            }
            else
            {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }

        // TODO Luigi: add lines here for stats 

        if(numMonoOptPoints + numStereoOptPoints < 50)
        {
            Verbose::PrintMess("LBA ERROR: KF " + to_string(pKFi->mnId) + " has only " + to_string(numMonoOptPoints) + " monocular and " + to_string(numStereoOptPoints) + " stereo points", Verbose::VERBOSITY_DEBUG);
        }
#endif 

        pKFi->SetPose(Tiw);
        pKFi->mnLBACount++;
    }

    //Points
    for(MapPointPtr pMPi : vpMPs)
    {
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMPi->mnId+maxKFid+1));
        pMPi->SetWorldPos(vPoint->estimate().cast<float>());
        pMPi->UpdateNormalAndDepth();

    }
    
#if USE_LINES_LOCAL_BA    
    //Lines
    for(MapLinePtr pMLi: vpMLs)
    {
        if(pMLi->isBad())
            continue;
             
        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pMLi->mnId+maxPointId+1));
        //if(vLine==NULL) continue; // check if we actually inserted the line in the graph 
        const Eigen::Matrix<double,6,1> line(vLine->estimate());
        Eigen::Vector3f pStartNew = (static_cast<const Eigen::Matrix<double,3,1> >(line.head(3))).cast<float>();
        Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Matrix<double,3,1> >(line.tail(3))).cast<float>();         

        pMLi->SetWorldEndPoints(pStartNew, pEndNew);
        pMLi->UpdateNormalAndDepth();

        if(vLine->isBad()) pMLi->SetBadFlag();
    }
#endif    
     
#if USE_OBJECTS_LOCAL_BA    
    //Objects
    for(MapObjectPtr pMObj: vpMOs)
    {    
        g2o::VertexSim3Expmap* vObject = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pMObj->mnId+maxLineId+1));
        if(vObject==NULL) continue; // check if we actually inserted the object in the graph 
        g2o::Sim3 correctedSow = vObject->estimate();   // Sow = [Row/s, tow; 0, 1]       
        Eigen::Matrix3d eigRow = correctedSow.rotation().toRotationMatrix();
        Eigen::Vector3d eigtow = correctedSow.translation();
        double scaleow = correctedSow.scale();

        Sophus::Sim3f Sow(Sophus::RxSO3d(scaleow, eigRow).cast<float>(), eigtow.cast<float>());  // Sow = [Row/s, tow; 0, 1] 
        //std::cout << "LBA - resulting Sow: " << Sow << std::endl; 

        pMObj->SetSim3Pose(Sow);

    }
#endif      
    
}


void Optimizer::MergeInertialBA(KeyFramePtr pCurrKF, KeyFramePtr pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses)
{
#if 1 //VERBOSE_BA
    std::cout << "******************************" << std::endl; 
    std::cout << "Optimizer::MergeInertialBA() " << std::endl; 
    std::cout << "******************************" << std::endl; 
#endif    
    const int Nd = 6;
    const unsigned long maxKFid = pCurrKF->mnId;

    vector<KeyFramePtr> vpOptimizableKFs;
    vpOptimizableKFs.reserve(2*Nd);

    // For cov KFS, inertial parameters are not optimized
    const int maxCovKF = 30;
    vector<KeyFramePtr> vpOptimizableCovKFs;
    vpOptimizableCovKFs.reserve(maxCovKF);

    // Add sliding window for current KF
    vpOptimizableKFs.push_back(pCurrKF);
    pCurrKF->mnBALocalForKF = pCurrKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    list<KeyFramePtr> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF=pCurrKF->mnId;
    }
    else
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    vpOptimizableKFs.push_back(pMergeKF);
    pMergeKF->mnBALocalForKF = pCurrKF->mnId;

    // Previous KFs
    for(int i=1; i<(Nd/2); i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    // We fix just once the old map
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pCurrKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pCurrKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Next KFs
    if(pMergeKF->mNextKF)
    {
        vpOptimizableKFs.push_back(pMergeKF->mNextKF);
        vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    }

    while(vpOptimizableKFs.size()<(2*Nd))
    {
        if(vpOptimizableKFs.back()->mNextKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by optimizable keyframes
    list<MapPointPtr> lLocalMapPoints;
    map<MapPointPtr,int> mLocalObs;

#if USE_LINES_MERGE_BA_INERTIAL
    // Optimizable lines seen by optimizable keyframes
    list<MapLinePtr> lLocalMapLines;
    map<MapLinePtr,int> mLocalObsLines;
#endif         

    for(int i=0; i<N; i++)
    {
        vector<MapPointPtr> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPointPtr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to lLocalMapPoints
            MapPointPtr pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pCurrKF->mnId)
                    {
                        mLocalObs[pMP]=1;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pCurrKF->mnId;
                    }
                    else {
                        mLocalObs[pMP]++;
                    }
        }

#if USE_LINES_MERGE_BA_INERTIAL
        vector<MapLinePtr> vpMLs = vpOptimizableKFs[i]->GetMapLineMatches();
        for(vector<MapLinePtr>::iterator vit=vpMLs.begin(), vend=vpMLs.end(); vit!=vend; vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one ML can not be added several times to lLocalMapLines
            MapLinePtr pML = *vit;
            if(pML)
                if(!pML->isBad())
                    if(pML->mnBALocalForKF!=pCurrKF->mnId)
                    {
                        mLocalObsLines[pML]=1;
                        lLocalMapLines.push_back(pML);
                        pML->mnBALocalForKF=pCurrKF->mnId;
                    }
                    else {
                        mLocalObsLines[pML]++;
                    }
        }        
#endif 

    }

    std::vector<std::pair<MapPointPtr, int>> pairs;
    pairs.reserve(mLocalObs.size());
    for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr) pairs.push_back(*itr);
    sort(pairs.begin(), pairs.end(),sortByVal);

#if USE_LINES_MERGE_BA_INERTIAL
    std::vector<std::pair<MapLinePtr, int>> pairsLines;
    pairsLines.reserve(mLocalObsLines.size());
    for (auto itr = mLocalObsLines.begin(); itr != mLocalObsLines.end(); ++itr) pairsLines.push_back(*itr);
    sort(pairsLines.begin(), pairsLines.end(),sortByValLines);
#endif         

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    int i=0;
    for(vector<pair<MapPointPtr,int>>::iterator lit=pairs.begin(), lend=pairs.end(); lit!=lend; lit++, i++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = lit->first->GetObservations();
        if(i>=maxCovKF)
            break;
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCurrKF->mnId && pKFi->mnBAFixedForKF!=pCurrKF->mnId) // If optimizable or already included...
            {
                pKFi->mnBALocalForKF=pCurrKF->mnId;
                if(!pKFi->isBad())
                {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }

#if USE_LINES_MERGE_BA_INERTIAL

    // Fixed Keyframes. Keyframes that see Local MapLines but that are not Local Keyframes
    i=0;
    for(vector<pair<MapLinePtr,int>>::iterator lit=pairsLines.begin(), lend=pairsLines.end(); lit!=lend; lit++, i++)
    {
        map<KeyFramePtr,tuple<int,int>> observations = lit->first->GetObservations();
        if(i>=maxCovKF)
            break;
        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCurrKF->mnId && pKFi->mnBAFixedForKF!=pCurrKF->mnId) // If optimizable or already included...
            {
                pKFi->mnBALocalForKF=pCurrKF->mnId;
                if(!pKFi->isBad())
                {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }
#endif // USE_LINES_MERGE_BA_INERTIAL


    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
#endif // USE_G2O_NEW

    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Set Local KeyFrame vertices
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local cov keyframes vertices
    int Ncov=vpOptimizableCovKFs.size();
    for(int i=0; i<Ncov; i++)
    {
        KeyFramePtr pKFi = vpOptimizableCovKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);
    for(int i=0;i<N;i++)
    {
        //cout << "inserting inertial edge " << i << endl;
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            MSG_WARN_STREAM("NO INERTIAL LINK TO PREVIOUS FRAME - KF id: " << pKFi->mnId);
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            // TODO Uncomment
            g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(rki);
            rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
            vear[i]->setInformation(InfoA);
            optimizer.addEdge(vear[i]);
        }
        else
            Verbose::PrintMess("ERROR building inertial edge", Verbose::VERBOSITY_NORMAL);
    }

    Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);


    // Set MapPoint vertices
    const int nExpectedSize = (N+Ncov+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);


#if USE_LINES_MERGE_BA_INERTIAL

    // Set MapLine vertices
    const int nExpectedSizeLines = (N+Ncov+lFixedKeyFrames.size())*lLocalMapLines.size();

    // Mono
    vector<EdgeLineMono*> vpEdgesMonoLines;
    vpEdgesMonoLines.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFMonoLines;
    vpEdgeKFMonoLines.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeMono;
    vpMapLineEdgeMono.reserve(nExpectedSizeLines);

    // Stereo
    vector<EdgeLineStereo*> vpEdgesStereoLines;
    vpEdgesStereoLines.reserve(nExpectedSizeLines);

    vector<KeyFramePtr> vpEdgeKFStereoLines;
    vpEdgeKFStereoLines.reserve(nExpectedSizeLines);

    vector<MapLinePtr> vpMapLineEdgeStereo;
    vpMapLineEdgeStereo.reserve(nExpectedSizeLines);

#endif 


    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const float thHuberLineMono = sqrt(5.991);  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float chi2LineMono2 = 5.991;    
    const float thHuberLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float chi2LineStereo2 = 9.49;    
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n
    const float chi2ObjectStereo2 = 3;    

    const unsigned long iniMPid = maxKFid*5;

    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        if (!pMP)
            continue;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if (!pKFi)
                continue;

            if ((pKFi->mnBALocalForKF!=pCurrKF->mnId) && (pKFi->mnBAFixedForKF!=pCurrKF->mnId))
                continue;

            if (pKFi->mnId>maxKFid){
                continue;
            }


            if(optimizer.vertex(id)==NULL || optimizer.vertex(pKFi->mnId)==NULL)
                continue;

            const int leftIndex = get<0>(mit->second);
            if(leftIndex == -1) {
                continue; 
            }

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];

                if(pKFi->mvuRight[leftIndex]<0) // Monocular observation
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // stereo observation
                {
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }


#if USE_LINES_MERGE_BA_INERTIAL

    const unsigned long maxPointId = iniMPid+1+MapPoint::GetCurrentMaxId(); 

    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;
        if (!pML)
            continue;

        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        Eigen::Vector3f posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);             
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());        
        unsigned long id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine);

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));

        const map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();

        // Create visual constraints
        for(map<KeyFramePtr,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if (!pKFi)
                continue;

            if ((pKFi->mnBALocalForKF!=pCurrKF->mnId) && (pKFi->mnBAFixedForKF!=pCurrKF->mnId))
                continue;

            if (pKFi->mnId>maxKFid){
                continue;
            }


            if(optimizer.vertex(id)==NULL || optimizer.vertex(pKFi->mnId)==NULL)
                continue;

            const int leftIndex = get<0>(mit->second);
            if(leftIndex == -1)
                continue; 

            if(!pKFi->isBad())
            {
                VertexPose* VP = dynamic_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
                if(VP == NULL)
                    continue;    
                const ImuCamPose& imuCamPose = VP->estimate();

                const float uRightLineStart = -1; 
                const float uRightLineEnd = -1; 
                if(!pKFi->mvuRightLineStart.empty() && (leftIndex < pKFi->mvuRightLineStart.size()))
                {
                    pKFi->mvuRightLineStart[leftIndex];
                    pKFi->mvuRightLineEnd[leftIndex];
                }

                const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[leftIndex];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);  

                // Monocular observation
    #if USE_LINES_STEREO_INERTIAL                
                if( uRightLineStart<0 || uRightLineEnd<0 ) 
    #endif                     
                {
                    EdgeLineMono* e = new EdgeLineMono(0);

                    e->setVertex(0, vertexLine);
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    
    #if !USE_NEW_LINE_INFORMATION_MAT   
                    const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
    #else
                    const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];

                    Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                    Eigen::Vector2d projMapP, projMapQ;
                    e->getMapLineProjections(projMapP, projMapQ);
                    Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                            klUn.startPointX,klUn.startPointY, 
                            klUn.endPointX,klUn.endPointY, 
                            lineRepresentation.nx, lineRepresentation.ny, 
                            projMapP, projMapQ);
                    e->setInformation(Info);
    #endif       

        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(thHuberLineMono);
        #endif    
                    e->setRobustKernel(rk);
                    
                    optimizer.addEdge(e);
                    
                    vpEdgesMonoLines.push_back(e);
                    vpEdgeKFMonoLines.push_back(pKFi);
                    vpMapLineEdgeMono.push_back(pML);
                }
                else // stereo observation
                {
                    EdgeLineStereo* e = new EdgeLineStereo(0);

                    e->setVertex(0, vertexLine);
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // the following two are actually derived/indirect observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                    const Eigen::Vector3f XSc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.startPointX,klUn.startPointY),pKFi->mvDepthLineStart[leftIndex] );
                    const Eigen::Vector3f XEc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.endPointX,klUn.endPointY),pKFi->mvDepthLineEnd[leftIndex] );
                    //std::cout << "XSc_backproj: " << XSc_backproj.transpose() << ", XEc_backproj: " << XEc_backproj.transpose() << std::endl; 
                    e->setBackprojections(XSc_backproj, XEc_backproj);
                    e->muWeigth = Optimizer::skMuWeightForLine3dDist;
                    
                    e->init(); // here we check the match between Bp and P (Bq and Q)


    #if !USE_NEW_LINE_INFORMATION_MAT                   
                    const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                    const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                    Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                    Info(0,0)*=invSigma2;
                    Info(1,1)*=invSigma2;
                    Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                    Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
    #else
                    const float sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];
                    Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                    Eigen::Vector2d projMapP, projMapQ;
                    Eigen::Vector3d mapP, mapQ;
                    e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                    Eigen::Vector3d &backprojP = e->XSbc;
                    Eigen::Vector3d &backprojQ = e->XEbc; 

                    Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                            klUn.startPointX,klUn.startPointY, 
                            klUn.endPointX,klUn.endPointY, 
                            lineRepresentation.nx, lineRepresentation.ny, 
                            projMapP, projMapQ);
    #if USE_NEW_LINE_INFORMATION_MAT_STEREO                
                    Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                    sigma2, klUn.octave,
                                    pKFi->fx, pKFi->fy, pKFi->mbfInv, 
                                    projMapP, projMapQ, 
                                    mapP, mapQ,
                                    backprojP, backprojQ);    
    #else
                    const float invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                    const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                    Info(2,2)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                    Info(3,3)=invSigma2LineError3D;//kInvSigma2PointLineDistance;
    #endif
                    
    #endif
                    
                    e->setInformation(Info);                    

            #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
            #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(thHuberLineStereo);
            #endif    
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);
                    
                    vpEdgesStereoLines.push_back(e);
                    vpEdgeKFStereoLines.push_back(pKFi);
                    vpMapLineEdgeStereo.push_back(pML);
                }
            }
        }
    }

#endif  // USE_LINES_MERGE_BA_INERTIAL


    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    vector<pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Mono2)
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


#if USE_LINES_MERGE_BA_INERTIAL
	vector<pair<KeyFramePtr,MapLinePtr> > vToEraseLines;
    vToEraseLines.reserve(vpEdgesMonoLines.size()+vpEdgesStereoLines.size());

    // Check line inlier observations
    // Mono Lines
    for(size_t i=0, iend=vpEdgesMonoLines.size(); i<iend;i++)
    {
        EdgeLineMono* e = vpEdgesMonoLines[i];
        MapLinePtr pML = vpMapLineEdgeMono[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>chi2LineMono2 || !e->areDepthsPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMonoLines[i];
            vToEraseLines.push_back(make_pair(pKFi,pML));
        }
    }

    // Stereo Lines
    for(size_t i=0, iend=vpEdgesStereoLines.size(); i<iend;i++)
    {
        EdgeLineStereo* e = vpEdgesStereoLines[i];
        MapLinePtr pML = vpMapLineEdgeStereo[i];

        if(pML->isBad())
            continue;

        if(e->zeroMuChi2()>chi2LineStereo2 || !e->areDepthsPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereoLines[i];
            vToEraseLines.push_back(make_pair(pKFi,pML));
        }
    }
#endif 

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // Erase point outliers
    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    
#if USE_LINES_MERGE_BA_INERTIAL
    // Erase line outliers
    if(!vToEraseLines.empty())
    {
        for(size_t i=0;i<vToEraseLines.size();i++)
        {
            KeyFramePtr pKFi = vToEraseLines[i].first;
            MapLinePtr pMLi = vToEraseLines[i].second;
            pKFi->EraseMapLineMatch(pMLi);
            pMLi->EraseObservation(pKFi);
        }
    }
#endif  // USE_LINES_MERGE_BA_INERTIAL

    // Recover optimized data
    //Keyframes
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
        corrPoses[pKFi] = g2oSiw;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));
        }
    }

    for(int i=0; i<Ncov; i++)
    {
        KeyFramePtr pKFi = vpOptimizableCovKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
        corrPoses[pKFi] = g2oSiw;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));
        }
    }

    //Points
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

#if USE_LINES_MERGE_BA_INERTIAL
    //Lines
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;
        g2o::VertexSBALine* vLine = static_cast<g2o::VertexSBALine*>(optimizer.vertex(pML->mnId+maxPointId+1));
        const Eigen::Vector3f pStartNew = (static_cast<const Eigen::Vector3d >(vLine->estimate().head(3))).cast<float>();
        const Eigen::Vector3f pEndNew   = (static_cast<const Eigen::Vector3d >(vLine->estimate().tail(3))).cast<float>();                
        pML->SetWorldEndPoints(pStartNew, pEndNew);
        pML->UpdateNormalAndDepth();

        if(vLine->isBad()) pML->SetBadFlag();
    }
#endif     

    pMap->IncreaseChangeIndex();
}

// OK Lines
int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
{
    g2o::SparseOptimizer optimizer;

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
#endif // USE_G2O_NEW

    optimizer.setVerbose(false);
    optimizer.setAlgorithm(solver);

    int nInitialMonoCorrespondences=0;
    int nInitialStereoCorrespondences=0;
    int nInitialCorrespondences=0;

    int nInitialLineCorrespondences=0;    

    // Set Frame vertex
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    const ImuCamPose& imuCamPose = VP->estimate();

    // Set MapPoint vertices
    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft!=-1);

    vector<EdgeMonoOnlyPose*> vpEdgesMono;
    vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeMono;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

#if USE_LINES_POSE_OPTIMIZATION     
    // Set MapLine vertices
    const int Nlines = pFrame->Nlines;
    const int NlinesLeft = pFrame->NlinesLeft;
    //const bool bLinesRight = (NlinesLeft!=-1);    
    
    vector<EdgeLineMonoOnlyPose*> vpEdgesLineMono;
    vector<EdgeLineStereoOnlyPose*> vpEdgesLineStereo;    
    vector<size_t> vnIndexEdgeLineMono;
    vector<size_t> vnIndexEdgeLineStereo;    
    vpEdgesLineMono.reserve(Nlines); 
    vpEdgesLineStereo.reserve(Nlines);
    vnIndexEdgeLineMono.reserve(Nlines);       
    vnIndexEdgeLineStereo.reserve(Nlines); 

    const float deltaLineMono   = sqrt(5.991);// chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float deltaLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs         
#endif    

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        // start points 
        for(int i=0; i<N; i++)
        {
            MapPointPtr pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                cv::KeyPoint kpUn;

                // Left monocular observation
                if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
                {
                    if(i < Nleft) // pair left-right
                        kpUn = pFrame->mvKeys[i];
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if(!bRight)
                {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if(bRight && i >= Nleft)
                {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }

        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

#if USE_LINES_POSE_OPTIMIZATION 

        for(int i=0; i<Nlines; i++)
        {
            MapLinePtr pML = pFrame->mvpMapLines[i];
            if(pML)
            {
                if( !pFrame->mpCamera2 ||                                  // Convential left image of pinhole cameras 
                    ( (pFrame->mpCamera2) && (i < pFrame->NlinesLeft) )    // Left image of fisheye cameras
                )
                {
                    // Monocular observation
            #if USE_LINE_STEREO            
                    if( (pFrame->mvuRightLineStart.empty()) || (pFrame->mvuRightLineStart[i]<0) || (pFrame->mvuRightLineEnd[i]<0) )
            #endif
                    {                    
                        nInitialLineCorrespondences++;
                        pFrame->mvbLineOutlier[i] = false;
                        pFrame->mvuNumLinePosOptFailures[i] = 0;

                        Eigen::Matrix<double,3,1> obs;
                        const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                        Eigen::Vector3f XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw); 
                        EdgeLineMonoOnlyPose* e = new EdgeLineMonoOnlyPose(XSw,XEw,0); // 0 = left camera index 

                        e->setVertex(0,VP);
                        e->setMeasurement(obs);

                        // Add here uncertainty
                        const float unc2 = 1.0f; //pFrame->mpCamera->uncertainty2(obs);

            #if !USE_NEW_LINE_INFORMATION_MAT   
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
            #else
                        const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave]*unc2;

                        Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                        Eigen::Vector2d projMapP, projMapQ;
                        e->getMapLineProjections(projMapP, projMapQ);
                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2,  
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                        e->setInformation(Info);

            #endif

        #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaLineMono);
        #endif      
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);

                        vpEdgesLineMono.push_back(e);
                        vnIndexEdgeLineMono.push_back(i);
                    }
        #if USE_LINE_STEREO                    
                    // Stereo observation
                    else 
                    {
                        nInitialLineCorrespondences++;
                        pFrame->mvbLineOutlier[i] = false;
                        pFrame->mvuNumLinePosOptFailures[i] = 0;

                        Eigen::Matrix<double,3,1> obs;
                        const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                        Eigen::Vector3f XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw);  
                        EdgeLineStereoOnlyPose* e = new EdgeLineStereoOnlyPose(XSw, XEw, 0); // 0 = left camera index 

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // the following two are actually derived/indirect observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                        const Eigen::Vector3f XSc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.startPointX,klUn.startPointY),pFrame->mvDepthLineStart[i] );
                        const Eigen::Vector3f XEc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.endPointX,klUn.endPointY),pFrame->mvDepthLineEnd[i] );
                        e->setBackprojections(XSc_backproj, XEc_backproj);
                        e->init();  

                        // Add here uncertainty
                        const float unc2 = 1.0f; //pFrame->mpCamera->uncertainty2(obs.head(2));

            #if !USE_NEW_LINE_INFORMATION_MAT                   
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)
                        const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                        Info(0,0)*=invSigma2;
                        Info(1,1)*=invSigma2;
                        Info(2,2)*=invSigma2PointLineDistance;
                        Info(3,3)*=invSigma2PointLineDistance;
            #else
                        const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave]*unc2;
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                        Eigen::Vector2d projMapP, projMapQ;
                        Eigen::Vector3d mapP, mapQ;
                        e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                        Eigen::Vector3d &backprojP = e->XSbc;
                        Eigen::Vector3d &backprojQ = e->XEbc; 

                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                #if USE_NEW_LINE_INFORMATION_MAT_STEREO                  
                        Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                        sigma2, klUn.octave,
                                        pFrame->fx, pFrame->fy, pFrame->mbfInv,
                                        projMapP, projMapQ, 
                                        mapP, mapQ,
                                        backprojP, backprojQ);          
                #else
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)       
                        const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                        
                        Info(2,2)=invSigma2PointLineDistance;
                        Info(3,3)=invSigma2PointLineDistance;
                #endif

            #endif                
                        e->setInformation(Info);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaLineStereo);
        #endif    
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);

                        vpEdgesLineStereo.push_back(e);
                        vnIndexEdgeLineStereo.push_back(i);
                    }
        #endif // #if USE_LINE_STEREO 
                }
    #if USE_LINES_RIGHT_PROJECTION                
                else         
                { // Right image SLAM 
                    nInitialLineCorrespondences++;
                    pFrame->mvbLineOutlier[i] = false;
                    pFrame->mvuNumLinePosOptFailures[i] = 0;

                    Eigen::Matrix<double,3,1> obs;
                    const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesRightUn[i-pFrame->NlinesLeft];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                    Eigen::Vector3f XSw, XEw;
                    pML->GetWorldEndPoints(XSw, XEw); 
                    EdgeLineMonoOnlyPose* e = new EdgeLineMonoOnlyPose(XSw,XEw,1); // 1 = right camera index 

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = 1.0f; //pFrame->mpCamera2->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(deltaLineMono);
        #endif      
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);

                    vpEdgesLineMono.push_back(e);
                    vnIndexEdgeLineMono.push_back(i);                    

                }
    #endif 

            } // if(pML)

        } // end lines loop

#endif // USE_LINES_POSE_OPTIMIZATION

    } // end lock MapPoint::mGlobalMutex


    KeyFramePtr pKF = pFrame->mpLastKeyFrame;
    VertexPose* VPk = new VertexPose(pKF);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);
    VertexVelocity* VVk = new VertexVelocity(pKF);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);
    VertexGyroBias* VGk = new VertexGyroBias(pKF);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);
    VertexAccBias* VAk = new VertexAccBias(pKF);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);

    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0,VGk);
    egr->setVertex(1,VG);
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0,VAk);
    ear->setVertex(1,VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    float chi2Mono[4]={12,7.5,5.991,5.991};                       // chi-squared 2 DOFs (first iterations have larger thresholds)
    float chi2Stereo[4]={15.6,9.8,7.815,7.815};                   // chi-squared 3 DOFs (first iterations have larger thresholds)
    
    //const float chi2LineMono[4]={5.991,5.991,5.991,5.991};      // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs for alpha=0.95 (Hartley Zisserman pg 119)
    const float chi2LineMono[4]={7.378,5.991,5.991,5.991};        // the first modified value 7.378 corresponds to 2 DOFs and alpha=0.975
    
    //const float chi2LineStereo[4]={9.49,9.49,9.49,9.49};        // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    const float chi2LineStereo[4]={11.14,9.49,9.49,9.49};          // the first modified value 11.14 corresponds to 4 DOFs and alpha=0.975 
    
    int its[4]={10,10,10,10};

    int nBad = 0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers = 0;

    int nBadLines = 0; 
    int nInliersLines = 0;

    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        float chi2close = 1.5*chi2Mono[it];

        nBadLines = 0;        
        nInliersLines = 0;

        // For monocular observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;

            if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadMono++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it==2)
                e->setRobustKernel(0);
        }

        // For stereo observations
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1); // not included in next optimization
                nBadStereo++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if(it==2)
                e->setRobustKernel(0);
        }


#if USE_LINES_POSE_OPTIMIZATION           
        
        // lines mono
        for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend; i++)
        {
            EdgeLineMonoOnlyPose* e = vpEdgesLineMono[i];

            const size_t idx = vnIndexEdgeLineMono[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineMono[it])
            {                
                //std::cout << "PoseOptimization() - mono line outlier error : " << chi2 << std::endl;
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] +=1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - mono line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);
                nInliersLines++; 
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        
#if USE_LINE_STEREO          
            
        // lines stereo
        for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend; i++)
        {
            EdgeLineStereoOnlyPose* e = vpEdgesLineStereo[i];

            const size_t idx = vnIndexEdgeLineStereo[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();                    
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineStereo[it])
            {                
                //std::cout << "PoseOptimization() - stereo line outlier error : " << chi2 << std::endl; 
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] += 1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - stereo line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);   
                nInliersLines++;                
            }

            if(it==2)
                e->setRobustKernel(0);
        }
                
#endif // USE_LINE_STEREO
        
#endif // USE_LINES_POSE_OPTIMIZATION


        nInliers = nInliersMono + nInliersStereo + Tracking::sknLineTrackWeigth*nInliersLines;
        nBad = nBadMono + nBadStereo; // just points 

        if(optimizer.edges().size()<10)
        {
#if VERBOSE_POSE_OPTIMIZATION 
            std::cout << "PoseInertialOptimizationLastKeyFrame() - stop optimization since remained only " << optimizer.edges().size() << " edges" << std::endl;
#endif            
            break;
        }

    }

#if VERBOSE_POSE_OPTIMIZATION    
    std::cout << "PoseInertialOptimizationLastKeyFrame() - points: " << nInitialCorrespondences <<" , outliers perc: " << 100*nBad/((float)nInitialCorrespondences) << std::endl; 
#if USE_LINES_POSE_OPTIMIZATION     
    std::cout << "PoseInertialOptimizationLastKeyFrame() - lines: " << nInitialLineCorrespondences <<" , outliers perc: " << 100*nBadLines/((float)nInitialLineCorrespondences) << std::endl; 
#endif
#endif

    // If not too much tracks, recover not too bad points
    if ((nInliers<30) && !bRecInit)
    {
        nBad=0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2()<chi2MonoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
        for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2()<chi2StereoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
    }

    // Recover optimized pose, velocity and biases
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

    // Recover Hessian, marginalize keyFrame states and generate new prior for frame
    Eigen::Matrix<double,15,15> H;
    H.setZero();

    H.block<9,9>(0,0)+= ei->GetHessian2();
    H.block<3,3>(9,9) += egr->GetHessian2();
    H.block<3,3>(12,12) += ear->GetHessian2();

    int tot_in = 0, tot_out = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

#if USE_LINES_POSE_OPTIMIZATION 
    int tot_lines_in = 0, tot_lines_out = 0;
    for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend; i++)
    {
        EdgeLineMonoOnlyPose* e = vpEdgesLineMono[i];

        const size_t idx = vnIndexEdgeLineMono[i];

        if(!pFrame->mvbLineOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_lines_in++;
        }
        else
            tot_lines_out++;
    }

    for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend; i++)
    {
        EdgeLineStereoOnlyPose* e = vpEdgesLineStereo[i];

        const size_t idx = vnIndexEdgeLineStereo[i];

        if(!pFrame->mvbLineOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_lines_in++;
        }
        else
            tot_lines_out++;
    }       
#endif     

    pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H);

    return (nInitialCorrespondences-nBad) + Tracking::sknLineTrackWeigth*(nInitialLineCorrespondences-nBadLines);
}

// OK Lines
int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
{
    g2o::SparseOptimizer optimizer;
#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
#endif // USE_G2O_NEW

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    int nInitialMonoCorrespondences=0;
    int nInitialStereoCorrespondences=0;
    int nInitialCorrespondences=0;

    int nInitialLineCorrespondences=0;     

    // Set Current Frame vertex
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    const ImuCamPose& imuCamPose = VP->estimate();

    // Set MapPoint vertices
    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;
    const bool bRight = (Nleft!=-1);

    vector<EdgeMonoOnlyPose*> vpEdgesMono;
    vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeMono;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

#if USE_LINES_POSE_OPTIMIZATION     
    // Set MapLine vertices
    const int Nlines = pFrame->Nlines;
    const int NlinesLeft = pFrame->NlinesLeft;
    //const bool bLinesRight = (NlinesLeft!=-1);    
    
    vector<EdgeLineMonoOnlyPose*> vpEdgesLineMono;
    vector<EdgeLineStereoOnlyPose*> vpEdgesLineStereo;    
    vector<size_t> vnIndexEdgeLineMono;
    vector<size_t> vnIndexEdgeLineStereo;    
    vpEdgesLineMono.reserve(Nlines); 
    vpEdgesLineStereo.reserve(Nlines);
    vnIndexEdgeLineMono.reserve(Nlines);       
    vnIndexEdgeLineStereo.reserve(Nlines); 

    const float deltaLineMono   = sqrt(5.991);// chi-squared 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float deltaLineStereo = sqrt(9.49); // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs         
#endif    

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        // start points 
        for(int i=0; i<N; i++)
        {
            MapPointPtr pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                cv::KeyPoint kpUn;
                // Left monocular observation
                if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
                {
                    if(i < Nleft) // pair left-right
                        kpUn = pFrame->mvKeys[i];
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if(!bRight)
                {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if(bRight && i >= Nleft)
                {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }

        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;


#if USE_LINES_POSE_OPTIMIZATION 

        for(int i=0; i<Nlines; i++)
        {
            MapLinePtr pML = pFrame->mvpMapLines[i];
            if(pML)
            {
                if( !pFrame->mpCamera2 ||                                  // Convential left image of pinhole cameras 
                    ( (pFrame->mpCamera2) && (i < pFrame->NlinesLeft) )    // Left image of fisheye cameras
                )
                {
                    // Monocular observation
            #if USE_LINE_STEREO            
                    if( (pFrame->mvuRightLineStart.empty()) || (pFrame->mvuRightLineStart[i]<0) || (pFrame->mvuRightLineEnd[i]<0) )
            #endif
                    {                    
                        nInitialLineCorrespondences++;
                        pFrame->mvbLineOutlier[i] = false;
                        pFrame->mvuNumLinePosOptFailures[i] = 0;

                        Eigen::Matrix<double,3,1> obs;
                        const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                        Eigen::Vector3f XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw); 
                        EdgeLineMonoOnlyPose* e = new EdgeLineMonoOnlyPose(XSw,XEw,0); // 0 = left camera index 

                        e->setVertex(0,VP);
                        e->setMeasurement(obs);

                        // Add here uncertainty
                        const float unc2 = 1; //pFrame->mpCamera->uncertainty2(obs);

            #if !USE_NEW_LINE_INFORMATION_MAT   
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
            #else
                        const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave]*unc2;

                        Eigen::Matrix2d Info = Eigen::Matrix2d::Zero(); 
                        Eigen::Vector2d projMapP, projMapQ;
                        e->getMapLineProjections(projMapP, projMapQ);
                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2,  
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                        e->setInformation(Info);

            #endif

        #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaLineMono);
        #endif      
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);

                        vpEdgesLineMono.push_back(e);
                        vnIndexEdgeLineMono.push_back(i);
                    }
        #if USE_LINE_STEREO                    
                    // Stereo observation
                    else 
                    {
                        nInitialLineCorrespondences++;
                        pFrame->mvbLineOutlier[i] = false;
                        pFrame->mvuNumLinePosOptFailures[i] = 0;

                        Eigen::Matrix<double,3,1> obs;
                        const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesUn[i];
                        Line2DRepresentation lineRepresentation;
                        Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                        obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                        Eigen::Vector3f XSw, XEw;
                        pML->GetWorldEndPoints(XSw, XEw);  
                        EdgeLineStereoOnlyPose* e = new EdgeLineStereoOnlyPose(XSw, XEw, 0); // 0 = left camera index 

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // the following two are actually derived/indirect observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                        const Eigen::Vector3f XSc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.startPointX,klUn.startPointY),pFrame->mvDepthLineStart[i] );
                        const Eigen::Vector3f XEc_backproj = imuCamPose.pCamera[0]->unprojectEigLinear(cv::Point2f(klUn.endPointX,klUn.endPointY),pFrame->mvDepthLineEnd[i] );
                        e->setBackprojections(XSc_backproj, XEc_backproj);
                        e->init();  

                        // Add here uncertainty
                        const float unc2 = 1.0; //pFrame->mpCamera->uncertainty2(obs.head(2));

            #if !USE_NEW_LINE_INFORMATION_MAT                   
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)
                        const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                        Info(0,0)*=invSigma2;
                        Info(1,1)*=invSigma2;
                        Info(2,2)*=invSigma2PointLineDistance;
                        Info(3,3)*=invSigma2PointLineDistance;
            #else
                        const float sigma2 = pFrame->mvLineLevelSigma2[klUn.octave]*unc2;
                        Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Zero();
                        Eigen::Vector2d projMapP, projMapQ;
                        Eigen::Vector3d mapP, mapQ;
                        e->getMapLineAndProjections(mapP, mapQ, projMapP, projMapQ);
                        Eigen::Vector3d &backprojP = e->XSbc;
                        Eigen::Vector3d &backprojQ = e->XEbc; 

                        Set2DLineInformationMat(Info(0,0),Info(1,1), sigma2, 
                                klUn.startPointX,klUn.startPointY, 
                                klUn.endPointX,klUn.endPointY, 
                                lineRepresentation.nx, lineRepresentation.ny, 
                                projMapP, projMapQ);
                #if USE_NEW_LINE_INFORMATION_MAT_STEREO                  
                        Set3DLineInformationMat(Info(2,2),Info(3,3), 
                                        sigma2, klUn.octave,
                                        pFrame->fx, pFrame->fy, pFrame->mbfInv,
                                        projMapP, projMapQ, 
                                        mapP, mapQ,
                                        backprojP, backprojQ);          
                #else
                        const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                        // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)       
                        const float invSigma2PointLineDistance = kInvSigma2PointLineDistance * invSigma2; //kInvSigma2PointLineDistance;                        
                        Info(2,2)=invSigma2PointLineDistance;
                        Info(3,3)=invSigma2PointLineDistance;
                #endif

            #endif                
                        e->setInformation(Info);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                        g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        rk->setDelta(deltaLineStereo);
        #endif    
                        e->setRobustKernel(rk);

                        optimizer.addEdge(e);

                        vpEdgesLineStereo.push_back(e);
                        vnIndexEdgeLineStereo.push_back(i);
                    }
        #endif // #if USE_LINE_STEREO 
                }
    #if USE_LINES_RIGHT_PROJECTION
                else         
                { // Right image SLAM 
                
                    nInitialLineCorrespondences++;
                    pFrame->mvbLineOutlier[i] = false;
                    pFrame->mvuNumLinePosOptFailures[i] = 0;

                    Eigen::Matrix<double,3,1> obs;
                    const cv::line_descriptor_c::KeyLine &klUn = pFrame->mvKeyLinesRightUn[i-pFrame->NlinesLeft];
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);

                    Eigen::Vector3f XSw, XEw;
                    pML->GetWorldEndPoints(XSw, XEw); 
                    EdgeLineMonoOnlyPose* e = new EdgeLineMonoOnlyPose(XSw,XEw,1); // 1 = right camera index 

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncertainty
                    const float unc2 = 1.0f; //pFrame->mpCamera2->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        #if USE_CAUCHY_KERNEL_FOR_LINES
                    g2o::RobustKernelCauchy* rk = new g2o::RobustKernelCauchy;
        #else 
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    rk->setDelta(deltaLineMono);
         #endif      
                    e->setRobustKernel(rk);

                    optimizer.addEdge(e);

                    vpEdgesLineMono.push_back(e);
                    vnIndexEdgeLineMono.push_back(i);      

                }
    #endif

            } // if(pML)

        } // end lines loop

#endif // USE_LINES_POSE_OPTIMIZATION

    } // end lock MapPoint::mGlobalMutex


    // Set Previous Frame Vertex
    Frame* pFp = pFrame->mpPrevFrame;

    VertexPose* VPk = new VertexPose(pFp);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    VertexVelocity* VVk = new VertexVelocity(pFp);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    VertexGyroBias* VGk = new VertexGyroBias(pFp);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    VertexAccBias* VAk = new VertexAccBias(pFp);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);

    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0,VGk);
    egr->setVertex(1,VG);
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0,VAk);
    ear->setVertex(1,VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    if (!pFp->mpcpi)
        Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId), Verbose::VERBOSITY_NORMAL);

    EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

    ep->setVertex(0,VPk);
    ep->setVertex(1,VVk);
    ep->setVertex(2,VGk);
    ep->setVertex(3,VAk);
    g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
    ep->setRobustKernel(rkp);
    rkp->setDelta(5);
    optimizer.addEdge(ep);

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={15.6f,9.8f,7.815f,7.815f};
    
    //const float chi2LineMono[4]={5.991,5.991,5.991,5.991};      // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs for alpha=0.95 (Hartley Zisserman pg 119)
    const float chi2LineMono[4]={7.378,5.991,5.991,5.991};        // the first modified value 7.378 corresponds to 2 DOFs and alpha=0.975
    
    //const float chi2LineStereo[4]={9.49,9.49,9.49,9.49};        // chi-squared 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    const float chi2LineStereo[4]={11.14,9.49,9.49,9.49};          // the first modified value 11.14 corresponds to 4 DOFs and alpha=0.975 
    

    const int its[4]={10,10,10,10};

    int nBad=0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers=0;

    int nBadLines = 0; 
    int nInliersLines = 0;

    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers=0;
        nInliersMono=0;
        nInliersStereo=0;
        float chi2close = 1.5*chi2Mono[it];

        nBadLines = 0;        
        nInliersLines = 0;        

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadMono++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it==2)
                e->setRobustKernel(0);

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadStereo++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if(it==2)
                e->setRobustKernel(0);
        }


#if USE_LINES_POSE_OPTIMIZATION           
        
        // lines mono
        for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend; i++)
        {
            EdgeLineMonoOnlyPose* e = vpEdgesLineMono[i];

            const size_t idx = vnIndexEdgeLineMono[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineMono[it])
            {                
                //std::cout << "PoseOptimization() - mono line outlier error : " << chi2 << std::endl;
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] +=1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - mono line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);
                nInliersLines++; 
            }

            if(it==2)
                e->setRobustKernel(0);
        }
        
#if USE_LINE_STEREO          
            
        // lines stereo
        for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend; i++)
        {
            EdgeLineStereoOnlyPose* e = vpEdgesLineStereo[i];

            const size_t idx = vnIndexEdgeLineStereo[i];

            if(pFrame->mvbLineOutlier[idx])
            {
                e->computeError();                    
            }

            const float chi2 = e->chi2();

            if(chi2>chi2LineStereo[it])
            {                
                //std::cout << "PoseOptimization() - stereo line outlier error : " << chi2 << std::endl; 
                
                pFrame->mvbLineOutlier[idx]=true;
                //pFrame->mvuNumLinePosOptFailures[idx] += 1;
                //pFrame->mvbLineOutlier[idx]=(pFrame->mvuNumLinePosOptFailures[idx] >= kNumMinPosOptFailuresForLineGettingOutlier);
                
                e->setLevel(1);
                nBadLines++;
            }
            else
            {
                //std::cout << "PoseOptimization() - stereo line *inlier* error : " << chi2 << std::endl;
                
                pFrame->mvuNumLinePosOptFailures[idx] = 0;
                pFrame->mvbLineOutlier[idx]=false;
                e->setLevel(0);   
                nInliersLines++;                
            }

            if(it==2)
                e->setRobustKernel(0);
        }
                
#endif // USE_LINE_STEREO
        
#endif // USE_LINES_POSE_OPTIMIZATION

        nInliers = nInliersMono + nInliersStereo + Tracking::sknLineTrackWeigth*nInliersLines;
        nBad = nBadMono + nBadStereo; // only points here

        if(optimizer.edges().size()<10)
        {
#if VERBOSE_POSE_OPTIMIZATION 
            std::cout << "PoseInertialOptimizationLastFrame() - stop optimization since remained only " << optimizer.edges().size() << " edges" << std::endl;
#endif                
            break;
        }
    }

#if VERBOSE_POSE_OPTIMIZATION    
    std::cout << "PoseInertialOptimizationLastFrame() - points: " << nInitialCorrespondences <<" , outliers perc: " << 100*nBad/((float)nInitialCorrespondences) << std::endl; 
#if USE_LINES_POSE_OPTIMIZATION     
    std::cout << "PoseInertialOptimizationLastFrame() - lines: " << nInitialLineCorrespondences <<" , outliers perc: " << 100*nBadLines/((float)nInitialLineCorrespondences) << std::endl; 
#endif
#endif

    // If not too much tracks, recover not too bad points
    if ((nInliers<30) && !bRecInit)
    {
        nBad=0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2()<chi2MonoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;

        }
        for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2()<chi2StereoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
    }

    nInliers = nInliersMono + nInliersStereo + Tracking::sknLineTrackWeigth*nInliersLines;


    // Recover optimized pose, velocity and biases
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

    // Recover Hessian, marginalize previous frame states and generate new prior for frame
    Eigen::Matrix<double,30,30> H;
    H.setZero();

    H.block<24,24>(0,0)+= ei->GetHessian();

    Eigen::Matrix<double,6,6> Hgr = egr->GetHessian();
    H.block<3,3>(9,9) += Hgr.block<3,3>(0,0);
    H.block<3,3>(9,24) += Hgr.block<3,3>(0,3);
    H.block<3,3>(24,9) += Hgr.block<3,3>(3,0);
    H.block<3,3>(24,24) += Hgr.block<3,3>(3,3);

    Eigen::Matrix<double,6,6> Har = ear->GetHessian();
    H.block<3,3>(12,12) += Har.block<3,3>(0,0);
    H.block<3,3>(12,27) += Har.block<3,3>(0,3);
    H.block<3,3>(27,12) += Har.block<3,3>(3,0);
    H.block<3,3>(27,27) += Har.block<3,3>(3,3);

    H.block<15,15>(0,0) += ep->GetHessian();

    int tot_in = 0, tot_out = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(15,15) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(15,15) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

#if USE_LINES_POSE_OPTIMIZATION 
    int tot_lines_in = 0, tot_lines_out = 0;
    for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend; i++)
    {
        EdgeLineMonoOnlyPose* e = vpEdgesLineMono[i];

        const size_t idx = vnIndexEdgeLineMono[i];

        if(!pFrame->mvbLineOutlier[idx])
        {
            H.block<6,6>(15,15) += e->GetHessian(); // pose of last frame 
            tot_lines_in++;
        }
        else
            tot_lines_out++;
    }

    for(size_t i=0, iend=vpEdgesLineStereo.size(); i<iend; i++)
    {
        EdgeLineStereoOnlyPose* e = vpEdgesLineStereo[i];

        const size_t idx = vnIndexEdgeLineStereo[i];

        if(!pFrame->mvbLineOutlier[idx])
        {
            H.block<6,6>(15,15) += e->GetHessian(); // pose of last frame 
            tot_lines_in++;
        }
        else
            tot_lines_out++;
    }       
#endif     

    H = Marginalize(H,0,14);

    pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H.block<15,15>(15,15));
    delete pFp->mpcpi;
    pFp->mpcpi = NULL;

    return (nInitialCorrespondences-nBad) + Tracking::sknLineTrackWeigth*(nInitialLineCorrespondences-nBadLines);
}




/// OK lines (used for inertial tracking)
void Optimizer::OptimizeEssentialGraph4DoF(Map* pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr, set<KeyFramePtr> > &LoopConnections)
{
    //typedef g2o::BlockSolver< g2o::BlockSolverTraits<4, 4> > BlockSolver_4_4;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

#ifdef USE_G2O_NEW        
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));        
#else
    g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif // USE_G2O_NEW

    optimizer.setAlgorithm(solver);

    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPointPtr> vpMPs = pMap->GetAllMapPoints();
#if USE_LINES_EG    
    const vector<MapLinePtr> vpMLs = pMap->GetAllMapLines();    
#endif     
#if USE_OBJECTS_EG
    const vector<MapObjectPtr> vpMOs= pMap->GetAllMapObjects();
#endif 

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);

    vector<VertexPose4DoF*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;
    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFramePtr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        VertexPose4DoF* V4DoF;

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            const g2o::Sim3 Swc = it->second.inverse();
            Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
            Eigen::Vector3d twc = Swc.translation();
            V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
        }
        else
        {
            Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

            vScw[nIDi] = Siw;
            V4DoF = new VertexPose4DoF(pKF);
        }

        if(pKF==pLoopKF)
            V4DoF->setFixed(true);

        V4DoF->setId(nIDi);
        V4DoF->setMarginalized(false);

        optimizer.addVertex(V4DoF);
        vpVertices[nIDi]=V4DoF;
    }
    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    // Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
    Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();
    matLambda(0,0) = 1e3; // roll freezed
    matLambda(1,1) = 1e3; // pitch freezed
#if 0    
    matLambda(0,0) = 1e3; // duplication, also signaled here: https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/351  
#endif     

    // Set Loop edges
    Edge4DoF* e_loop;
    for(map<KeyFramePtr, set<KeyFramePtr> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFramePtr> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];

        for(set<KeyFramePtr>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sij = Siw * Sjw.inverse();
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3) = 1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));

            e->information() = matLambda;
            e_loop = e;
            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // 1. Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFramePtr pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Siw;

        // Use noncorrected poses for posegraph edges
        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Siw = iti->second;
        else
            Siw = vScw[nIDi];

        // 1.1.0 Spanning tree edge
        KeyFramePtr pParentKF = static_cast<KeyFramePtr>(NULL);
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj =  vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3)=1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.1.1 Inertial edges
        KeyFramePtr prevKF = pKF->mPrevKF;
        if(prevKF)
        {
            int nIDj = prevKF->mnId;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(prevKF);

            if(itj!=NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj =  vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3)=1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.2 Loop edges
        const set<KeyFramePtr> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFramePtr>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFramePtr pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Swl;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Swl = itl->second.inverse();
                else
                    Swl = vScw[pLKF->mnId].inverse();

                g2o::Sim3 Sil = Siw * Swl;
                Eigen::Matrix4d Til;
                Til.block<3,3>(0,0) = Sil.rotation().toRotationMatrix();
                Til.block<3,1>(0,3) = Sil.translation();
                Til(3,3) = 1.;

                Edge4DoF* e = new Edge4DoF(Til);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }
        }

        // 1.3 Covisibility graph edges
        const vector<KeyFramePtr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFramePtr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFramePtr pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && pKFn!=prevKF && pKFn!=pKF->mNextKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Swn;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Swn = itn->second.inverse();
                    else
                        Swn = vScw[pKFn->mnId].inverse();

                    g2o::Sim3 Sin = Siw * Swn;
                    Eigen::Matrix4d Tin;
                    Tin.block<3,3>(0,0) = Sin.rotation().toRotationMatrix();
                    Tin.block<3,1>(0,3) = Sin.translation();
                    Tin(3,3) = 1.;
                    Edge4DoF* e = new Edge4DoF(Tin);
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    e->information() = matLambda;
                    optimizer.addEdge(e);
                }
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
        Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
        Eigen::Vector3d ti = Vi->estimate().tcw[0];

        g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri,ti,1.);
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();

        Sophus::SE3d Tiw(CorrectedSiw.rotation(),CorrectedSiw.translation());
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPointPtr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;

        KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }

#if USE_LINES_EG
    // Correct lines. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMLs.size(); i<iend; i++)
    {
        MapLinePtr pML = vpMLs[i];

        if(pML->isBad())
            continue;

        int nIDr;

        KeyFramePtr pRefKF = pML->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Vector3f P3DSw, P3DEw;
        pML->GetWorldEndPoints(P3DSw, P3DEw);             
        
        Eigen::Matrix<double,3,1> eigP3DSw = P3DSw.cast<double>();
        Eigen::Matrix<double,3,1> eigP3DEw = P3DEw.cast<double>();
        
        Eigen::Matrix<double,3,1> eigCorrectedP3DSw = correctedSwr.map(Srw.map(eigP3DSw));
        Eigen::Matrix<double,3,1> eigCorrectedP3DEw = correctedSwr.map(Srw.map(eigP3DEw));
        
        Eigen::Vector3f correctedP3DSw = eigCorrectedP3DSw.cast<float>();
        Eigen::Vector3f correctedP3DEw = eigCorrectedP3DEw.cast<float>();
        
        pML->SetWorldEndPoints(correctedP3DSw, correctedP3DEw);        
        
        pML->UpdateNormalAndDepth();        
    }
#endif     

#if USE_OBJECTS_EG
    // Correct objects. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMOs.size(); i<iend; i++)
    {
        MapObjectPtr pMO = vpMOs[i];

        if(pMO->isBad())
            continue;

        int nIDr;

        KeyFramePtr pRefKF = pMO->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];
        
        // Map to non-corrected camera (convert Sim(3) Srw to an SE(3) transformation)
        const double scw = Srw.scale();
        Eigen::Matrix3f Rcw = Srw.rotation().matrix().cast<float>();
        Eigen::Vector3f tcw = (Srw.translation()/scw).cast<float>();
        
        const double scale = pMO->GetScale();
        Eigen::Matrix3f Rwo = pMO->GetInverseRotation();
        Eigen::Vector3f two = pMO->GetInverseTranslation();

        const Eigen::Matrix3f Rco = Rcw*Rwo;
        const Eigen::Vector3f tco = Rcw*two+tcw;  

        // Backproject using corrected camera (convert Sim(3) correctedSwr to an SE(3) transformation)
        const double swc = correctedSwr.scale();        
        Eigen::Matrix3f Rwc = correctedSwr.rotation().matrix().cast<float>();
        Eigen::Vector3f twc = (correctedSwr.translation()/swc).cast<float>();
        
        const Eigen::Matrix3f RwoNew = Rwc*Rco;
        const Eigen::Vector3f twoNew = Rwc*tco+twc;
        
        pMO->SetSim3InversePose(RwoNew, twoNew, scale);   // keep the original object to world scale  
    }
#endif 

    pMap->IncreaseChangeIndex();
}

} // namespace PLVS2
