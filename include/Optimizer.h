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
*/


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"


#ifdef USE_G2O_NEW
#include "Thirdparty/g2o_new/install/include/g2o/types/sim3/types_seven_dof_expmap.h"
#else
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#endif 

namespace PLVS
{

class KeyFrame;
class MapPoint;
class MapLine;
class MapObject; 

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
                                 const vector<MapLinePtr> &vpMLines, const vector<MapObjectPtr > &vpMObjects, 
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFramePtr pKF, bool *pbStopFlag, Map *pMap);
    
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFramePtr pLoopKF, KeyFramePtr pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFramePtr, set<KeyFramePtr> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFramePtr pKF1, KeyFramePtr pKF2, std::vector<MapPointPtr> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace PLVS

#endif // OPTIMIZER_H
