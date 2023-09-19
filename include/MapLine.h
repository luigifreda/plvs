/*
 * This file is part of PLVS
 * Copyright (C) 2018  Luigi Freda <luigifreda at gmail dot com>
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

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "BoostArchiver.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <memory>

namespace PLVS
{

class KeyFrame;
class Map;
class Frame;

//typedef Eigen::Matrix<int,5,1>    Vector5i;
//typedef Eigen::Matrix<double,6,1> Vector6d;
//typedef Eigen::Matrix<double,6,6> Matrix6d;

class MapLine
{

public: 
    typedef std::shared_ptr<MapLine> Ptr;    
    typedef std::shared_ptr<const MapLine> ConstPtr;   
    
public:

    MapLine(const cv::Mat& p3DStart, const cv::Mat& p3DEnd, Map* pMap, KeyFramePtr pRefKF);
    MapLine(const cv::Mat& p3DStart, const cv::Mat& p3DEnd, Map* pMap, Frame* pFrame, const int &idxF);
    
    ~MapLine();

    
    void SetWorldEndPoints(const cv::Mat &PosStart, const cv::Mat &PosEnd);
    void GetWorldEndPoints(cv::Mat &PosStart, cv::Mat &PosEnd);    
    
    void SetWorldPosStart(const cv::Mat &Pos);
    cv::Mat GetWorldPosStart();
    
    void SetWorldPosEnd(const cv::Mat &Pos);
    cv::Mat GetWorldPosEnd();

    cv::Mat GetNormal();
    KeyFramePtr GetReferenceKeyFrame();
    
    float GetLength(); 

    std::map<KeyFramePtr,size_t> GetObservations();
    int Observations();

    bool AddObservation(const KeyFramePtr& pKF,size_t idx);
    void EraseObservation(const KeyFramePtr& pKF);

    int GetIndexInKeyFrame(const KeyFramePtr& pKF);
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

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFramePtr& pKF);
    int PredictScale(const float &currentDist, Frame* pF);    
    
public: 
    
    static long unsigned int GetCurrentMaxId();   

public:
    // for serialization
    MapLine();
    
private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);    
    
public: 
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjStartX;
    float mTrackProjStartY;
    float mTrackStartDepth;
    //float mTrackStartDepthR;
    float mTrackProjStartXR;
    //float mTrackProjStartYR;
    
    float mTrackProjEndX;
    float mTrackProjEndY;
    float mTrackEndDepth;
    //float mTrackEndDepthR;
    float mTrackProjEndXR;
    //float mTrackProjEndYR;
    
    float mTrackProjMiddleX;
    float mTrackProjMiddleY;
    float mTrackMiddleDepth;
    float mTrackMiddleDepthR;
    float mTrackProjMiddleXR;
    //float mTrackProjMiddleYR;
    
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosStartGBA;
    cv::Mat mPosEndGBA;
    long unsigned int mnBAGlobalForKF;
    
    unsigned int muNumLineBAFailures;
    
    static std::mutex mGlobalMutex;    

protected:

    // Position in absolute coordinates
    cv::Mat mWorldPosStart; //  3x1 mat
    cv::Mat mWorldPosEnd;   //  3x1 mat
    
    float mfLength; // [m]
    
    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFramePtr,size_t> mObservations;  
     
    // Mean viewing direction
    cv::Mat mNormalVector;
     
    // Reference KeyFrame
    KeyFramePtr mpRefKF;
    
    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    MapLinePtr mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    // Tracking counters
    int mnVisible;
    int mnFound;
         
    // Best descriptor to fast matching
    cv::Mat          mDescriptor;
     
    Map* mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
    
};

} //namespace 

#endif // MAP_LINE_H
