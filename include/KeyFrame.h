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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Utils.h"
#include "Geom2DUtils.h"

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include <mutex>
#include "BoostArchiver.h"
#include "Pointers.h"


namespace PLVS
{

//class MapPoint;
//class MapLine;
//class MapObject;
//class Frame;

class Map;
class KeyFrameDatabase;

class KeyFrame
{
    
public:
        
    static const float kDeltaTheta; 
    static const float kDeltaD;
    static const float kTgViewZAngleMin;
    static const float kDeltaZForCheckingViewZAngleMin;
    
    static float skFovCenterDistance; // [m]
    
public: 

    typedef std::shared_ptr<KeyFrame> Ptr;    
    typedef std::shared_ptr<const KeyFrame> ConstPtr;
    
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);
    ~KeyFrame();

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    cv::Mat GetFovCenter();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(const KeyFramePtr& pKF, const int &weight);
    void EraseConnection(const KeyFramePtr& pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFramePtr> GetConnectedKeyFrames();
    std::vector<KeyFramePtr> GetVectorCovisibleKeyFrames();
    std::vector<KeyFramePtr> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFramePtr> GetCovisiblesByWeight(const int &w);
    int GetWeight(const KeyFramePtr& pKF);

    // Spanning tree functions
    void AddChild(const KeyFramePtr& pKF);
    void EraseChild(const KeyFramePtr& pKF);
    void ChangeParent(KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetChilds();
    KeyFramePtr GetParent();
    bool hasChild(const KeyFramePtr& pKF);

    // Loop Edges
    void AddLoopEdge(KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetLoopEdges();

    // MapPoint observation functions
    void AddMapPoint(const MapPointPtr& pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPointPtr& pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPointPtr pMP);
    std::set<MapPointPtr> GetMapPoints();
    std::vector<MapPointPtr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPointPtr GetMapPoint(const size_t &idx);
    
    // MapLine observation functions
    void AddMapLine(const MapLinePtr& pML, const size_t &idx);
    void EraseMapLineMatch(const size_t &idx);
    void EraseMapLineMatch(MapLinePtr& pMP);
    void ReplaceMapLineMatch(const size_t &idx, MapLinePtr pMP);
    std::set<MapLinePtr> GetMapLines();
    std::vector<MapLinePtr> GetMapLineMatches();
    int TrackedMapLines(const int &minObs);
    MapLinePtr GetMapLine(const size_t &idx);    
    
    // MapObject observation functions
    void AddMapObject(const MapObjectPtr& pMObj);
    void EraseMapObjectMatch(const size_t &id);    
    void EraseMapObjectMatch(MapObjectPtr& pMObj);
    void ReplaceMapObjectMatch(MapObjectPtr pMObjToReplace, MapObjectPtr pMObj);
    std::set<MapObjectPtr > GetMapObjects();
    std::vector<MapObjectPtr > GetMapObjectMatches();
    int TrackedMapObjects(const int &minObs);
    //MapObjectPtr GetMapObject(const size_t &id);  
    
    

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(const int& i);
    
    // KeyLine functions
    bool UnprojectStereoLine(const int& i, cv::Mat& p3DStart, cv::Mat& p3DEnd);
    std::vector<size_t> GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, const float& dtheta = kDeltaTheta, const float& dd = kDeltaD) const;
    std::vector<size_t> GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, const float& dtheta = kDeltaTheta, const float& dd = kDeltaD) const; 
    void GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, vector<size_t>& vIndices) const;

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }
  
    static bool lId(KeyFramePtr& pKF1, KeyFramePtr& pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

  bool mbVisited;

 public:
    // for serialization
    KeyFrame();  // Default constructor for serialization, need to deal with const member
    void SetORBvocabulary(ORBVocabulary* porbv) { mpORBvocabulary = porbv; }

 private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template < class Archive >
    void serialize(Archive& ar, const unsigned int version);

 public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;
    
    const int mnLineDGridCols;
    const int mnLineThetaGridRows;
    const float mfLineGridElementThetaInv;
    const float mfLineGridElementDInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mbfInv, mb, mThDepth;
    const cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;
    
    
    // Number of KeyLines
    const int Nlines;

    // KeyLines, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::line_descriptor_c::KeyLine> mvKeyLines;
    const std::vector<cv::line_descriptor_c::KeyLine> mvKeyLinesUn;
    /*const*/ std::vector<float> mvuRightLineStart;
    /*const*/ std::vector<float> mvDepthLineStart;
    /*const*/ std::vector<float> mvuRightLineEnd;
    /*const*/ std::vector<float> mvDepthLineEnd;
    const cv::Mat mLineDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;
    
    const int mnLineScaleLevels;
    const float mfLineScaleFactor;
    const float mfLineLogScaleFactor;    
    const std::vector<float> mvLineScaleFactors;
    const std::vector<float> mvLineLevelSigma2;
    const std::vector<float> mvLineInvLevelSigma2;    

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const int mnMaxDiag;
    const cv::Mat mK;

    bool mbFixed; // to be considered as fixed (freezed) during optimization?

    unsigned int mnLBACount = 0;    
    
    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization
    
    cv::Mat fovCw; // FOV Center (along optical axis, at distance 0.5*perceptionRange from camera center)
    float mMedianDepth;

    // MapPoints associated to keypoints
    std::vector<MapPointPtr> mvpMapPoints;
    
    // Maplines associated to keylines
    std::vector<MapLinePtr> mvpMapLines;
    
    // MapObjects
    std::vector<MapObjectPtr> mvpMapObjects;    

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;
    std::vector< std::vector <std::vector<size_t> > > mLineGrid;

    std::map<KeyFramePtr,int> mConnectedKeyFrameWeights;
    std::vector<KeyFramePtr> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFramePtr mpParent;
    std::set<KeyFramePtr> mspChildrens;
    std::set<KeyFramePtr> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexLineFeatures;
    std::mutex mMutexObjects;
    
    
    static const std::vector<size_t> kEmptyVecSizet; 
};

} //namespace PLVS

#endif // KEYFRAME_H
