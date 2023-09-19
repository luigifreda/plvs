/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified by Luigi Freda (2017-present)
 *  - added map lines and their covariances 
 *  - added map objects 
 *  - added parameters for optimization 
 *  - added compatibility with new g2o
*/


#include "Optimizer.h"

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
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#endif

#include<Eigen/StdVector>

#include "g2o/types_sba_line.h"
#include "g2o/types_six_dof_expmap2.h"
#include "g2o/types_seven_dof_expmap2.h"

#include "MapObject.h"
#include "Converter.h"
#include "Utils.h"
#include "Geom2DUtils.h"

#include<mutex>

#define USE_LINES 1                                  // set to zero to completely exclude lines from optimization
#define USE_LINE_STEREO             (1 && USE_LINES)
#define USE_LINE_PRIOR_BA           (0 && USE_LINES) // <- not used, it was used just for testing!

#define USE_LINES_POSE_OPTIMIZATION (1 && USE_LINES)
#define USE_LINES_LOCAL_BA          (1 && USE_LINES)
#define USE_LINES_BA                (1 && USE_LINES) 
#define USE_LINES_EG                (1 && USE_LINES) 

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

namespace PLVS
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

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    long unsigned int maxPointId = 0; 
    long unsigned int maxLineId = 0;

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        //vSE3->setFixed(pKF->mnId==0);
        vSE3->setFixed(pKF->mbFixed);
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);  // chi-square 2 DOFs
    const float thHuber3D = sqrt(7.815); // chi-square 3 DOFs
    
    const float thHuberLineMono = sqrt(5.99);    // chi-square 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49);  // chi-square 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    
    const float thHuberObjectTimesSigma = sqrt(3);  // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    // Set MapPoint vertices
    for(size_t i=0; i<vpMPoints.size(); i++)
    {
        MapPointPtr pMP = vpMPoints[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId+maxKFid+1;
        //if(id>maxPointId) maxPointId = id; 
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        
        g2o::OptimizableGraph::Vertex* vertexPoint = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));

        const map<KeyFramePtr,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;

            g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));
            if(vertexKF == NULL)
                    continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

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

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else
            {
                Eigen::Matrix<double,3,1> obs;

#if !USE_RGBD_POINT_REPROJ_ERR                    
                const float kp_ur = pKF->mvuRight[mit->second];
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
    //cout << "end inserting MPs" << endl;

    maxPointId = maxKFid+1+MapPoint::GetCurrentMaxId(); 
    
#if USE_LINES_BA    // ---------------------------------------------------------
    // Set MapLine vertices
    for(size_t i=0; i<vpMLines.size(); i++)
    {
        MapLinePtr pML = vpMLines[i];
        if(pML->isBad())
            continue;
        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        cv::Mat posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);          
                
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());
        const int id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine);

        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,size_t> observations = pML->GetObservations();
        //if(observations.size() < kNumMinLineObservationsForBA) continue;        

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFramePtr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;
            
            g2o::OptimizableGraph::Vertex* vertexKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId));                        
            if(vertexKF == NULL)
                continue;              

            nEdges++;
            
            const cv::line_descriptor_c::KeyLine &klUn = pKF->mvKeyLinesUn[mit->second];
            Line2DRepresentation lineRepresentation;
            Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                
            // Monocular observation
#if USE_LINE_STEREO                
            if( (pKF->mvuRightLineStart[mit->second]<0) || (pKF->mvuRightLineEnd[mit->second]<0) )
#endif            
            {
                Eigen::Matrix<double,3,1> obs;
                obs << lineRepresentation.nx, lineRepresentation.ny, (-lineRepresentation.d);                    

                g2o::EdgeSE3ProjectLine* e = new g2o::EdgeSE3ProjectLine();
                    
                //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setVertex(0, vertexLine);
                e->setVertex(1, vertexKF);  
                e->setMeasurement(obs);
                
                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;                
                
#if !USE_NEW_LINE_INFORMATION_MAT   
                const float& invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
#else
                const float& sigma2 = pKF->mvLineLevelSigma2[klUn.octave];

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
            }
#if USE_LINE_STEREO              
            else
            {
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
                e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pKF->mvDepthLineStart[mit->second]);
                e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pKF->mvDepthLineEnd[mit->second]);
                        
                e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 
                e->mu = Optimizer::skMuWeightForLine3dDist;
                
                e->init(); // here we check the match between Bp and P (Bq and Q)
                
#if !USE_NEW_LINE_INFORMATION_MAT                   
                const float &invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
                // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                
                const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;
                Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                Info(0,0)*=invSigma2;
                Info(1,1)*=invSigma2;
                Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
#else
                const float& sigma2 = pKF->mvLineLevelSigma2[klUn.octave];
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
                const float &invSigma2 = pKF->mvLineInvLevelSigma2[klUn.octave];
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
            }
#endif            
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
        Eigen::Matrix<double,3,3> Row = Converter::toMatrix3d(pMObj->GetRotation());
        Eigen::Matrix<double,3,1> tow = Converter::toVector3d(pMObj->GetTranslation());
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

            cv::Mat Tko = observation.GetSE3();
            Eigen::Matrix<double,3,3> Rko = Converter::toMatrix3d(Tko.rowRange(0,3).colRange(0,3));
            Eigen::Matrix<double,3,1> tko = Converter::toVector3d(Tko.rowRange(0,3).col(3));                
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
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

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
        if(nLoopKF==0)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

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

        if(nLoopKF==0)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
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

        const cv::Mat pStartNew = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(vLine->estimate().head(3)));
        const cv::Mat pEndNew   = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(vLine->estimate().tail(3)));
        
        if(nLoopKF==0)
        {
            pML->SetWorldEndPoints(pStartNew, pEndNew);
            pML->UpdateNormalAndDepth();
        }
        else
        {
            pML->mPosStartGBA.create(3,1,CV_32F);
            pML->mPosEndGBA.create(3,1,CV_32F);
            pStartNew.copyTo(pML->mPosStartGBA);
            pEndNew.copyTo(pML->mPosEndGBA);
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
        
        cv::Mat SowNew = Converter::toCvSim3(eigRow,eigtow,scaleow);  // Sow = [Row/s, tow; 0, 1] 
        
        if(nLoopKF==0)
        {
            pMObj->SetSim3Pose(SowNew);
            //pMObj->UpdateNormalAndDepth();
        }
        else
        {
            pMObj->mSowGBA.create(4,4,CV_32F);
            SowNew.copyTo(pMObj->mSowGBA);
            pMObj->mnBAGlobalForKF = nLoopKF;
        }
    }
#endif    

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

    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;
    int nInitialLineCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);
    
    //g2o::OptimizableGraph::Vertex* vertexSE3 = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0));   

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

#if !USE_RGBD_POINT_REPROJ_ERR    
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
#else
    vector<g2o::EdgeRgbdSE3ProjectXYZOnlyPose*> vpEdgesStereo;
#endif 
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);    // chi-square 2 DOFs
    const float deltaStereo = sqrt(7.815);  // chi-square 3 DOFs
    
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

    
    const float deltaLineMono   = sqrt(5.991);// chi-square 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float deltaLineStereo = sqrt(9.49); // chi-square 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
        
#endif
    
    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    // start points 
    for(int i=0; i<N; i++)
    {
        MapPointPtr pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            if(pFrame->mvuRight[i]<0)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                //e->setVertex(0, vertexSE3);
                
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->fx = pFrame->fx;
                e->fy = pFrame->fy;
                e->cx = pFrame->cx;
                e->cy = pFrame->cy;
                cv::Mat Xw = pMP->GetWorldPos();
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

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
                const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
          
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
                cv::Mat Xw = pMP->GetWorldPos();
                
                e->Xw[0] = Xw.at<float>(0);
                e->Xw[1] = Xw.at<float>(1);
                e->Xw[2] = Xw.at<float>(2);

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vnIndexEdgeStereo.push_back(i);
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
            // Monocular observation
#if USE_LINE_STEREO            
            if( (pFrame->mvuRightLineStart[i]<0) || (pFrame->mvuRightLineEnd[i]<0) )
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

                //cv::Mat XSw = pML->GetWorldPosStart();
                //cv::Mat XEw = pML->GetWorldPosEnd();
                cv::Mat XSw, XEw;
                pML->GetWorldEndPoints(XSw, XEw);                   
                
                e->XSw[0] = XSw.at<float>(0);
                e->XSw[1] = XSw.at<float>(1);
                e->XSw[2] = XSw.at<float>(2);

                e->XEw[0] = XEw.at<float>(0);
                e->XEw[1] = XEw.at<float>(1);
                e->XEw[2] = XEw.at<float>(2);
                
#if !USE_NEW_LINE_INFORMATION_MAT   
                const float& invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
#else
                const float& sigma2 = pFrame->mvLineLevelSigma2[klUn.octave];
                
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
                                
                //const cv::Mat XSw = pML->GetWorldPosStart();
                //const cv::Mat XEw = pML->GetWorldPosEnd();
                cv::Mat XSw, XEw;
                pML->GetWorldEndPoints(XSw, XEw);                    
                
                e->XSw = Eigen::Vector3d(XSw.at<float>(0), XSw.at<float>(1), XSw.at<float>(2));

                e->XEw = Eigen::Vector3d(XEw.at<float>(0), XEw.at<float>(1), XEw.at<float>(2));  
                
                // the following two are actually derived observations (using also depth measurements) but we keep them cached inside the edge for simplicity
                e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pFrame->mvDepthLineStart[i] );
                e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pFrame->mvDepthLineEnd[i] );
                        
                //e->lineLenghtInv = 1.0/pML->GetLength();
                e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the  length of the 3D detected line 
                
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
                const float& sigma2 = pFrame->mvLineLevelSigma2[klUn.octave];
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
                const float& invSigma2 = pFrame->mvLineInvLevelSigma2[klUn.octave];
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
#endif  // #if USE_LINE_STEREO 
        
        }
        
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
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};            // chi-square 2 DOFs
    const float chi2Stereo[4]={7.815,7.815,7.815,7.815};          // chi-square 3 DOFs
    const float chi2LineMono[4]={5.991,5.991,5.991,5.991};        // chi-square 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley Zisserman pg 119)
    const float chi2LineStereo[4]={9.49,9.49,9.49,9.49};          // chi-square 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs 
    const int its[4]={10,10,10,10};    

    int nBad=0;
    int nBadLines = 0; 
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        nBadLines = 0;
        
        // points  mono 
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

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

            if(chi2>chi2Stereo[it])
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
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    pFrame->SetPose(pose);

    return (nInitialCorrespondences-nBad) + Tracking::sknLineTrackWeigth*(nInitialLineCorrespondences-nBadLines);
}


/// < < 

void Optimizer::LocalBundleAdjustment(KeyFramePtr pKF, bool* pbStopFlag, Map* pMap)
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

    const vector<KeyFramePtr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFramePtr pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints and MapLines seen in Local KeyFrames
    list<MapPointPtr> lLocalMapPoints;
    list<MapLinePtr> lLocalMapLines;
    list<MapObjectPtr> lLocalMapObjects;    
    
    for(list<KeyFramePtr>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPointPtr> vpMPs = (*lit)->GetMapPointMatches();
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

#if USE_LINES_LOCAL_BA     
        
        vector<MapLinePtr> vpMLs = (*lit)->GetMapLineMatches();
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
        
#if USE_OBJECTS_LOCAL_BA
        
        vector<MapObjectPtr> vpMObjs = (*lit)->GetMapObjectMatches();
        for(vector<MapObjectPtr>::iterator vit=vpMObjs.begin(), vend=vpMObjs.end(); vit!=vend; vit++)
        {
            MapObjectPtr pMObj = *vit;
            if(pMObj)
                if(!pMObj->isBad())
                    if(pMObj->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapObjects.push_back(pMObj);
                        pMObj->mnBALocalForKF=pKF->mnId;
                    }
        }   
        
#endif 
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFramePtr> lFixedCameras;
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

#if USE_LINES_LOCAL_BA 
    
    // Fixed Keyframes. Keyframes that see Local MapLines but that are not Local Keyframes
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    
#endif
    
#if USE_OBJECTS_LOCAL_BA
        
    // Fixed Keyframes. Keyframes that see Local MapObjects but that are not Local Keyframes
    for(list<MapObjectPtr >::iterator lit=lLocalMapObjects.begin(), lend=lLocalMapObjects.end(); lit!=lend; lit++)
    {
        map<KeyFramePtr,ObjectObservation> observations = (*lit)->GetObservations();
        for(map<KeyFramePtr,ObjectObservation>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }        
        
#endif            

    for(auto it=lFixedCameras.begin(); it!=lFixedCameras.end(); it++)
    {
        //std::cout << "LocalBundleAdjustment - setting KF " << (*it)->mnId << " fixed" << std::endl; 
        (*it)->mnLBACount++;
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

    optimizer.setAlgorithm(solver);
    /*if(numObjects>0)
    {
        solver->setUserLambdaInit(1e-16);           
    }*/

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        //vSE3->setFixed(pKFi->mnId==0);
        vSE3->setFixed(pKFi->mbFixed);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFramePtr>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFramePtr pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    unsigned long maxPointId = maxKFid+1+MapPoint::GetCurrentMaxId(); 
    
    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    // point mono
    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
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

    unsigned long maxLineId = maxPointId+1+MapLine::GetCurrentMaxId();     
    
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
    
#endif // USE_LINES_LOCAL_BA
    
    
#if USE_OBJECTS_LOCAL_BA
    
    
    const int nExpectedSizeObjects = lLocalMapObjects.size()*lLocalKeyFrames.size();

    vector<g2o::EdgeSim3SE3*> vpEdgesObject;
    vpEdgesObject.reserve(nExpectedSizeObjects);
    
    vector<KeyFramePtr> vpEdgeKFObject; 
    vpEdgeKFObject.reserve(nExpectedSizeObjects);

    vector<MapObjectPtr > vpMapObjectEdge; 
    vpMapObjectEdge.reserve(nExpectedSizeObjects);    
    
    std::vector<double> vEdgesObjectSquaredErrors;
    vEdgesObjectSquaredErrors.reserve(nExpectedSizeObjects);        
     

#endif // USE_OBJECTS_LOCAL_BA     

    
    const float thHuberMono = sqrt(5.991);      // chi-square 2 DOFS 
    const float thHuberStereo = sqrt(7.815);    // chi-square 3 DOFS
    const float thHuberLineMono = sqrt(5.991);  // chi-square 2 2D-perpendicular-line-distances = 2 DOFs  (Hartley pg 119)
    const float thHuberLineStereo = sqrt(9.49); // chi-square 2 2D-perpendicular-line-distances + 2 3D-perpendicular-line-distances = 4 DOFs
    const float thHuberObjectTimesSigma = sqrt(3); // we estimate sigma2 = E[ek^2] and use it to normalize the object error, n=3 is used for rejecting outliers that have ek^2/sigma2 > n

    
// -----------------------------------------------------------------------------

#if VERBOSE_LOCAL_BA     
    int numConsideredPoints = 0; 
    int numConsideredPointEdges = 0; 
#endif    
        
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
#if VERBOSE_LOCAL_BA        
        numConsideredPoints++;
#endif

        g2o::OptimizableGraph::Vertex* vertexPoint = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFramePtr,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad())
            {                
                
                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                if(vertexKFi == NULL)
                        continue;
                                    
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
                    
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

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);    
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
    
#if VERBOSE_LOCAL_BA                     
                    numConsideredPointEdges++;
#endif
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;

#if !USE_RGBD_POINT_REPROJ_ERR                      
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();
#else
                    const float kpDelta = pKFi->mvDepth[mit->second];
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
                    
#if VERBOSE_LOCAL_BA                     
                    numConsideredPointEdges++;
#endif
                }
            }
        }
        
        
    }

#if USE_LINES_LOCAL_BA    // ---------------------------------------------------
    
#if VERBOSE_LOCAL_BA       
    int numConsideredLines = 0; 
    int numConsideredLineEdges = 0; 
#endif    
    
    for(list<MapLinePtr>::iterator lit=lLocalMapLines.begin(), lend=lLocalMapLines.end(); lit!=lend; lit++)
    {
        MapLinePtr pML = *lit;
        
        const map<KeyFramePtr,size_t> observations = pML->GetObservations();
        //if(observations.size() < kNumMinLineObservationsForBA)  continue;
        
        g2o::VertexSBALine* vLine = new g2o::VertexSBALine();
        cv::Mat posStart, posEnd;
        pML->GetWorldEndPoints(posStart, posEnd);          
        vLine->setEstimate(Converter::toVector6d(posStart,posEnd));
        vLine->setInitialLength(pML->GetLength());
        // vLine->P = posStart.cast<double>();
        // vLine->Q = posEnd.cast<double>();
        int id = pML->mnId+maxPointId+1;
        vLine->setId(id);
        vLine->setMarginalized(true);
        optimizer.addVertex(vLine); 
        
        g2o::OptimizableGraph::Vertex* vertexLine = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
                
#if VERBOSE_LOCAL_BA        
        numConsideredLines++;
#endif
        
#if CHECK_LINE_VALID_OBSERVATIONS        
        int numValidObservations = 0; 
#endif        
        
        //Set edges
        for(map<KeyFramePtr,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad())
            {                

                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));                        
                if(vertexKFi == NULL)
                        continue;                
                                    
                const cv::line_descriptor_c::KeyLine &klUn = pKFi->mvKeyLinesUn[mit->second];
                Line2DRepresentation lineRepresentation;
                Geom2DUtils::GetLine2dRepresentationNoTheta(klUn.startPointX,klUn.startPointY,klUn.endPointX,klUn.endPointY, lineRepresentation);
                    
                // Monocular observation
#if USE_LINE_STEREO                
                if( (pKFi->mvuRightLineStart[mit->second]<0) || (pKFi->mvuRightLineEnd[mit->second]<0) )
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
                    
                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;                    

#if !USE_NEW_LINE_INFORMATION_MAT   
                    const float& invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
#else
                    const float& sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];

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
#if VERBOSE_LOCAL_BA                       
                    numConsideredLineEdges++;
#endif                    
                    
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
                    e->XSbc = e->camBackProject(Eigen::Vector2d(klUn.startPointX,klUn.startPointY),pKFi->mvDepthLineStart[mit->second]);
                    e->XEbc = e->camBackProject(Eigen::Vector2d(klUn.endPointX,klUn.endPointY),pKFi->mvDepthLineEnd[mit->second]);
                        
                    e->lineLenghtInv = 1.0/(e->XSbc - e->XEbc).norm(); // use the length of the 3D detected line 
                    e->mu = Optimizer::skMuWeightForLine3dDist;
                    
                    e->init();
                    
#if !USE_NEW_LINE_INFORMATION_MAT                   
                    const float &invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
                    // N.B: we modulate all the information matrix with invSigma2 (so that all the components of the line error are weighted uniformly according to the detection uncertainty)                    
                    const float invSigma2LineError3D = skInvSigma2LineError3D * invSigma2; //kInvSigma2PointLineDistance;                    
                    Eigen::Matrix<double,4,4> Info = Eigen::Matrix<double,4,4>::Identity();
                    Info(0,0)*=invSigma2;
                    Info(1,1)*=invSigma2;
                    Info(2,2)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;
                    Info(3,3)*=invSigma2LineError3D;//kInvSigma2PointLineDistance;            
#else
                    const float& sigma2 = pKFi->mvLineLevelSigma2[klUn.octave];
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
                    const float &invSigma2 = pKFi->mvLineInvLevelSigma2[klUn.octave];
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
                    
#if VERBOSE_LOCAL_BA                    
                    numConsideredLineEdges++;
#endif                    
                    
#if CHECK_LINE_VALID_OBSERVATIONS                        
                    numValidObservations++;
#endif
                    
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
    
#if VERBOSE_LOCAL_BA     
    int numConsideredObjects = 0; 
    int numConsideredObjectEdges = 0; 
#endif    

    bool bFixScale = false;
    
    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
    
    for(list<MapObjectPtr >::iterator lit=lLocalMapObjects.begin(), lend=lLocalMapObjects.end(); lit!=lend; lit++)
    {
        MapObjectPtr pMObj = *lit;
        g2o::VertexSim3Expmap* vObject = new g2o::VertexSim3Expmap();
        const Eigen::Matrix<double,3,3> Row = Converter::toMatrix3d(pMObj->GetRotation());
        const Eigen::Matrix<double,3,1> tow = Converter::toVector3d(pMObj->GetTranslation());
        const double objectScale = pMObj->GetScale();
        g2o::Sim3 Sow(Row,tow,1./objectScale); // Sow = [Row/s, tow; 0, 1]  
        //std::cout << "LBA - Sow: " << Converter::toCvMat(Sow) << std::endl; 
        vObject->setEstimate(Sow);
        int id = pMObj->mnId+maxLineId+1;
        vObject->setId(id);
        vObject->setMarginalized(true);
        vObject->_fix_scale = bFixScale;        
        optimizer.addVertex(vObject);
        
#if VERBOSE_LOCAL_BA        
        numConsideredObjects++;
#endif

        g2o::OptimizableGraph::Vertex* vertexObject = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id));
        
        const map<KeyFramePtr,ObjectObservation> observations = pMObj->GetObservations();

        //Set edges
        for(map<KeyFramePtr,ObjectObservation>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad())
            {                                
                g2o::OptimizableGraph::Vertex* vertexKFi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                if(vertexKFi == NULL) continue;
                                    
                const ObjectObservation& observation = mit->second;
                
                //if(!observation.bFromKF) continue; 
                
                const cv::Mat Tko = observation.GetSE3(); // from object to keyframe               
                const Eigen::Matrix<double,3,3> Rko = Converter::toMatrix3d(Tko.rowRange(0,3).colRange(0,3));
                const Eigen::Matrix<double,3,1> tko = Converter::toVector3d(Tko.rowRange(0,3).col(3));                
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
                numConsideredObjectEdges++;                
#endif                
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
    
    
    if(pbStopFlag)
        if(*pbStopFlag)
        {
            return;
        }

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            e->setLevel(1);
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
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }
    
#if USE_LINES_LOCAL_BA    
    for(size_t i=0, iend=vpEdgesLineMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectLine* e = vpEdgesLineMono[i];
        MapLinePtr pML = vpMapLineEdgeMono[i];

        if(pML->isBad())
            continue;

        if(e->chi2()>5.991 || !e->areDepthsPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }
    
#if USE_LINE_STEREO       
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
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }
#endif
    
#endif // USE_LINES_LOCAL_BA
    
    
#if USE_OBJECTS_LOCAL_BA    
    for(size_t i=0, iend=vpEdgesObject.size(); i<iend;i++)
    {
        g2o::EdgeSim3SE3* e = vpEdgesObject[i];
        MapObjectPtr pMObj = vpMapObjectEdge[i];

        if(pMObj->isBad())
            continue;

        if(e->chi2()>3) // err_k^2/sigma_err^2 > 3
        {
            e->setLevel(1);
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

    int numPointEdgesOutliers = 0; 
    // Check inlier observations for points     
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
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
        KeyFramePtr pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        //if(vSE3==NULL) continue;// check if we actually inserted the keyframe in the graph             
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
        pKF->mnLBACount++;
        //std::cout << "LocalBundleAdjustement() - adjusted KF " << pKFi->mnId << " (fixed: "<< pKFi->mbFixed << ")"<< std::endl; 
    }

    //Points
    for(list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPointPtr pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        //if(vPoint==NULL) continue; // check if we actually inserted the point in the graph 
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
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
        const cv::Mat pStartNew = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(line.head(3)));
        const cv::Mat pEndNew   = Converter::toCvMat(static_cast<const Eigen::Matrix<double,3,1> >(line.tail(3)));

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

        cv::Mat Sow = Converter::toCvSim3(eigRow,eigtow,scaleow);  // Sow = [Row/s, tow; 0, 1] 
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
    
}


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
    solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_7_3>(g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>()));        
#else    
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPointPtr> vpMPoints = pMap->GetAllMapPoints();

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
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
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
            g2o::OptimizableGraph::Vertex* vj = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj));
            g2o::OptimizableGraph::Vertex* vi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi));
            if( (vj==NULL) || (vi==NULL) ) continue; 
            
            e->setVertex(1, vj);
            e->setVertex(0, vi);
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
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
            
            g2o::OptimizableGraph::Vertex* vj = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj));
            g2o::OptimizableGraph::Vertex* vi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi));
            if( (vj==NULL) || (vi==NULL) ) continue; 
            
            e->setVertex(1, vj);
            e->setVertex(0, vi);
            e->setMeasurement(Sji);

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
                
                g2o::OptimizableGraph::Vertex* vLKF = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId));
                g2o::OptimizableGraph::Vertex* vi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi));
                if( (vLKF==NULL) || (vi==NULL) ) continue; 
            
                el->setVertex(1, vLKF);
                el->setVertex(0, vi);
                
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFramePtr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFramePtr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFramePtr pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
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
                    
                    g2o::OptimizableGraph::Vertex* vKFn = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId));
                    g2o::OptimizableGraph::Vertex* vi = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi));
                    if( (!vKFn) || (!vi) ) continue; 

                    en->setVertex(1, vKFn);
                    en->setVertex(0, vi);
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFramePtr pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        if(VSim3==NULL) continue; 
        
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPoints.size(); i<iend; i++)
    {
        MapPointPtr pMP = vpMPoints[i];

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
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw);

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
            nIDr = pRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3DSw, P3DEw;
        pML->GetWorldEndPoints(P3DSw, P3DEw);             
        
        Eigen::Matrix<double,3,1> eigP3DSw = Converter::toVector3d(P3DSw);
        Eigen::Matrix<double,3,1> eigP3DEw = Converter::toVector3d(P3DEw);
        
        Eigen::Matrix<double,3,1> eigCorrectedP3DSw = correctedSwr.map(Srw.map(eigP3DSw));
        Eigen::Matrix<double,3,1> eigCorrectedP3DEw = correctedSwr.map(Srw.map(eigP3DEw));
        
        cv::Mat cvCorrectedP3DSw = Converter::toCvMat(eigCorrectedP3DSw);
        cv::Mat cvCorrectedP3DEw = Converter::toCvMat(eigCorrectedP3DEw);
        
        pML->SetWorldEndPoints(cvCorrectedP3DSw, cvCorrectedP3DEw);        
        
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
            nIDr = pRefKF->mnId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];
        
        // Map to non-corrected camera (convert Sim(3) Srw to an SE(3) transformation)
        const double scw = Srw.scale();
        Eigen::Matrix3f Rcw = Srw.rotation().matrix().cast<float>();
        Eigen::Vector3f tcw = (Srw.translation()/scw).cast<float>();
        
        const double scale = pMO->GetScale();
        Eigen::Matrix3f Rwo = Converter::toMatrix3f(pMO->GetInverseRotation());
        Eigen::Vector3f two = Converter::toVector3f(pMO->GetInverseTranslation());

        const Eigen::Matrix3f Rco = Rcw*Rwo;
        const Eigen::Vector3f tco = Rcw*two+tcw;  

        // Backproject using corrected camera (convert Sim(3) correctedSwr to an SE(3) transformation)
        const double swc = correctedSwr.scale();        
        Eigen::Matrix3f Rwc = correctedSwr.rotation().matrix().cast<float>();
        Eigen::Vector3f twc = (correctedSwr.translation()/swc).cast<float>();
        
        const Eigen::Matrix3f RwoNew = Rwc*Rco;
        const Eigen::Vector3f twoNew = Rwc*tco+twc;
        
        pMO->SetSim3InversePose(Converter::toCvMat(RwoNew), Converter::toCvMat(twoNew), scale);   // keep the original object to world scale  
    }
#endif 
    
}

int Optimizer::OptimizeSim3(KeyFramePtr pKF1, KeyFramePtr pKF2, vector<MapPointPtr> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
{

#if VERBOSE
    std::cout << "Optimizer::OptimizeSim3() " << std::endl; 
#endif      
    
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg* solver;
    
#ifdef USE_G2O_NEW        
    solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));        
#else   
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
#endif

    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPointPtr> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPointPtr pMP1 = vpMapPoints1[i];
        MapPointPtr pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
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
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

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
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPointPtr>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
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
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPointPtr>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}


} //namespace PLVS
