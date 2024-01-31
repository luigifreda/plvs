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


#ifndef MAP_LINE_H
#define MAP_LINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

#include "BoostArchiver.h"

namespace PLVS2
{

class KeyFrame;
class Map;
class Frame;

class MapLine
{
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);

public: 
    typedef std::shared_ptr<MapLine> Ptr;    
    typedef std::shared_ptr<const MapLine> ConstPtr;   
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapLine();

    MapLine(const Eigen::Vector3f& PosStart, const Eigen::Vector3f& PosEnd, KeyFramePtr pRefKF, Map* pMap);
    MapLine(const Eigen::Vector3f& PosStart, const Eigen::Vector3f& PosEnd, Map* pMap, Frame* pFrame, const int &idxF);
    MapLine(const double invDepthStart, cv::Point2f uv_initStart, const double invDepthEnd, cv::Point2f uv_initEnd, KeyFramePtr pRefKF, KeyFramePtr pHostKF, Map* pMap);

    ~MapLine();

    
    void SetWorldEndPoints(const Eigen::Vector3f &PosStart, const Eigen::Vector3f &PosEnd);
    void GetWorldEndPoints(Eigen::Vector3f &PosStart, Eigen::Vector3f &PosEnd);  
    void GetWorldEndPointsAndLength(Eigen::Vector3f &PosStart, Eigen::Vector3f &PosEnd, float& length);             
    
    void SetWorldPosStart(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPosStart();
    
    void SetWorldPosEnd(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPosEnd();

    Eigen::Vector3f GetNormal();
    KeyFramePtr GetReferenceKeyFrame();
    
    float GetLength(); 

    std::map<KeyFramePtr,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(const KeyFramePtr& pKF,int idx);
    void EraseObservation(const KeyFramePtr& pKF);

    std::tuple<int,int> GetIndexInKeyFrame(const KeyFramePtr& pKF);
    bool IsInKeyFrame(const KeyFramePtr& pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapLinePtr& pML);    
    MapLinePtr GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();
    void UdateLength(); 
        
    void SetNormalVector(Eigen::Vector3f& normal);

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFramePtr& pKF);
    int PredictScale(const float &currentDist, Frame* pF);    
    
public: 
    
    static long unsigned int GetCurrentMaxId();   

public:
    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(set<KeyFramePtr>& spKF,set<MapLinePtr>& spMP);
    void PostLoad(map<long unsigned int, KeyFramePtr>& mpKFid, map<long unsigned int, MapLinePtr>& mpMPid);   

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking (these are assumed to be undistorted in order to be used to extract a line representation)
    float mTrackProjStartX = -1;
    float mTrackProjStartY = -1;
    float mTrackStartDepth = -1;
    float mTrackStartDepthR = -1;
    float mTrackProjStartXR = -1; 
    float mTrackProjStartYR = -1; 
    
    float mTrackProjEndX = -1;
    float mTrackProjEndY = -1;
    float mTrackEndDepth = -1;
    float mTrackEndDepthR = -1;
    float mTrackProjEndXR = -1;
    float mTrackProjEndYR = -1;
    
    // float mTrackProjMiddleX = -1;  // not used now
    // float mTrackProjMiddleY = -1;  // not used now
    // float mTrackMiddleDepth = -1;  // not used now
    // float mTrackMiddleDepthR = -1; // not used now
    // float mTrackProjMiddleXR = -1; // not used now
    // float mTrackProjMiddleYR = -1; // not used now
    
    bool mbTrackInView = false, mbTrackInViewR = false;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Eigen::Vector3f mPosStartGBA;
    Eigen::Vector3f mPosEndGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;
    
    unsigned int muNumLineBAFailures;

    // Variable used by merging
    Eigen::Vector3f mPosStartMerge;
    Eigen::Vector3f mPosEndMerge;
    Eigen::Vector3f mNormalVectorMerge;


    // For inverse depth optimization
    double mInvDepthStart;
    double mInitUStart;
    double mInitVStart;
    double mInvDepthEnd;
    double mInitUEnd;
    double mInitVEnd;
    KeyFramePtr mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:    

     // Position in absolute coordinates
    Eigen::Vector3f mWorldPosStart; //  3x1 mat
    Eigen::Vector3f mWorldPosEnd;   //  3x1 mat
    
    float mfLength; // [m]

     // Keyframes observing the line and associated index in keyframe
     std::map<KeyFramePtr,std::tuple<int,int> > mObservations;   // KF -> <left idx, right idx>
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFramePtr mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapLine from memory)
     bool mbBad;
     MapLinePtr mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;
};

} //namespace PLVS2

#endif // MAP_LINE_H
