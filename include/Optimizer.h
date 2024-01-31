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
*/


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

//#include "Map.h"
//#include "MapPoint.h"
//#include "MapLine.h"
//#include "KeyFrame.h"
#include "LoopClosing.h"
//#include "Frame.h"
#include "Pointers.h"

#include <math.h>

#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#endif 

namespace PLVS2
{

class KeyFrame;
class MapPoint;
class MapLine;
class MapObject; 
class LoopClosing;
class Frame;
class Map; 

class Optimizer
{
public:
    
    static const int kNumMinLineObservationsForBA;
    static const int kNumMinBAFailuresForErasingMapLine;
    static const int kNumMinPosOptFailuresForLineGettingOutlier;
    static const float kSigmaPointLinePrior; 
    static const float kInvSigma2PointLinePrior; 
    static const float kSigmaPointLineDistance; 
    static const float kInvSigma2PointLineDistance; 
    static const float kMinLineSegmentLength; 
    static const float kInvMinLineSegmentLength; 
    
    static float skSigmaZFactor; 
    static float skMuWeightForLine3dDist; 
    static float skSigmaLineError3D;     
    static float skInvSigma2LineError3D;

public:

    void static BundleAdjustment(const std::vector<KeyFramePtr> &vpKF, const std::vector<MapPointPtr> &vpMPoints,
                                 const std::vector<MapLinePtr> &vpMLines, const std::vector<MapObjectPtr > &vpMObjects, 
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(KeyFramePtr pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_MLs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);

    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr, set<KeyFramePtr> > &LoopConnections,
                                       const bool &bFixScale);

    void static OptimizeEssentialGraph(KeyFramePtr pCurKF, std::vector<KeyFramePtr> &vpFixedKFs, std::vector<KeyFramePtr> &vpFixedCorrectedKFs,
                                       std::vector<KeyFramePtr> &vpNonFixedKFs, std::vector<MapPointPtr> &vpNonCorrectedMPs, 
                                       std::vector<MapLinePtr> &vpNonCorrectedMLs, vector<MapObjectPtr> &vpNonCorrectedMOs);

    // For inertial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr , set<KeyFramePtr> > &LoopConnections);


    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFramePtr pKF1, KeyFramePtr pKF2, std::vector<MapPointPtr> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);

    // For inertial systems

    void static LocalInertialBA(KeyFramePtr pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);
    void static MergeInertialBA(KeyFramePtr pCurrKF, KeyFramePtr pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFramePtr pMainKF, std::vector<KeyFramePtr> vpAdjustKF, std::vector<KeyFramePtr> vpFixedKF, bool *pbStopFlag, bool useLinesOrObject);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Condition block element (start:end,start:end). Fill with zeros.
    static Eigen::MatrixXd Condition(const Eigen::MatrixXd &H, const int &start, const int &end);
    // Remove link between element 1 and 2. Given elements 1,2 and 3 must define the whole matrix.
    static Eigen::MatrixXd Sparsify(const Eigen::MatrixXd &H, const int &start1, const int &end1, const int &start2, const int &end2);

    // Inertial pose-graph
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} //namespace PLVS2

#endif // OPTIMIZER_H
