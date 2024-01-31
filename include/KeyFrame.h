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


#ifndef KEYFRAME_H
#define KEYFRAME_H

//#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>
#include <unordered_set>
#include "BoostArchiver.h"
#include "Pointers.h"

namespace PLVS2
{

class Map;
//class MapPoint;
//class Frame;
class KeyFrameDatabase;

class GeometricCamera;

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

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);
    ~KeyFrame();

    // Pose functions
    void SetPose(const Sophus::SE3f &Tcw);
    void SetVelocity(const Eigen::Vector3f &Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();

    Eigen::Vector3f GetFovCenter();

    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(const KeyFramePtr& pKF, const int &weight);
    void EraseConnection(const KeyFramePtr& pKF);

    void UpdateConnections(bool upParent=true);
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
    void SetFirstConnection(bool bFirst);

    // Loop Edges
    void AddLoopEdge(KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetLoopEdges();

    // Merge Edges
    void AddMergeEdge(KeyFramePtr pKF);
    std::set<KeyFramePtr> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(const MapPointPtr& pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPointPtr& pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPointPtr pMP);
    std::set<MapPointPtr> GetMapPoints();
    std::unordered_set<MapPointPtr> GetMapPointsUnordered();
    std::vector<MapPointPtr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPointPtr GetMapPoint(const size_t &idx);
    
    // MapLine observation functions
    void AddMapLine(const MapLinePtr& pML, const size_t &idx);
    void EraseMapLineMatch(const size_t &idx);
    void EraseMapLineMatch(MapLinePtr& pMP);
    void ReplaceMapLineMatch(const size_t &idx, MapLinePtr pMP);
    std::set<MapLinePtr> GetMapLines();
    std::unordered_set<MapLinePtr> GetMapLinesUnordered();
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
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;
    bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

    // KeyLine functions
    bool UnprojectStereoLine(const int& i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd);
    bool UnprojectStereoLineFishEye(const int &i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd);    
    std::vector<size_t> GetLineFeaturesInArea(const float &xs, const float &ys, const float &xe, const float &ye, const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const bool bRight = false) const;
    std::vector<size_t> GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, const float& dtheta = kDeltaTheta, const float& dd = kDeltaD, const bool bRight = false) const; 
    void GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, std::vector<size_t>& vIndices, const bool bRight = false) const;

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

    bool mbVisited = false;

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias &b);
    Eigen::Vector3f GetGyroBias();
    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    bool ProjectPointDistort(MapPointPtr pMP, cv::Point2f &kp, float &u, float &v);
    bool ProjectPointUnDistort(MapPointPtr pMP, cv::Point2f &kp, float &u, float &v);

    void PreSave(set<KeyFramePtr>& spKF,set<MapPointPtr>& spMP, set<GeometricCamera*>& spCam);
    void PostLoad(map<long unsigned int, KeyFramePtr>& mpKFid, map<long unsigned int, MapPointPtr>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId);


    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    bool bImu;

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
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

    //Number of optimizations by BA(amount of iterations in BA)
    long unsigned int mnNumberOfOpt;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    long unsigned int mnMergeQuery;
    int mnMergeWords;
    float mMergeScore;
    long unsigned int mnPlaceRecognitionQuery;
    int mnPlaceRecognitionWords;
    float mPlaceRecognitionScore;

    bool mbCurrentPlaceRecognition;


    // Variables used by loop closing
    Sophus::SE3f mTcwGBA;
    Sophus::SE3f mTcwBefGBA;
    Eigen::Vector3f mVwbGBA;
    Eigen::Vector3f mVwbBefGBA;
    IMU::Bias mBiasGBA;
    long unsigned int mnBAGlobalForKF;

    // Variables used by merging
    Sophus::SE3f mTcwMerge;
    Sophus::SE3f mTcwBefMerge;
    Sophus::SE3f mTwcBefMerge;
    Eigen::Vector3f mVwbMerge;
    Eigen::Vector3f mVwbBefMerge;
    IMU::Bias mBiasMerge;
    long unsigned int mnMergeCorrectedForKF;
    long unsigned int mnMergeForKF;
    float mfScaleMerge;
    long unsigned int mnBALocalForMerge;

    float mfScale;

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
    Sophus::SE3f mTcp;

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
    cv::Mat mK; // mirror of the Eigen version mK_

    bool mbFixed = false; // to be considered as fixed (freezed) during optimization?

    // Preintegrated IMU measurements from previous keyframe
    KeyFramePtr mPrevKF;
    KeyFramePtr mNextKF;

    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;


    unsigned int mnOriginMapId;

    string mNameFile;

    int mnDataset;

    std::vector <KeyFramePtr> mvpLoopCandKFs;
    std::vector <KeyFramePtr> mvpMergeCandKFs;

    //bool mbHasHessian;
    //cv::Mat mHessianPose;

    unsigned int mnLBACount = 0;    

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // sophus poses
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    Eigen::Vector3f Ow;
    Eigen::Vector3f Cw; // Stereo middle point. Only for visualization

    Eigen::Vector3f fovCw; // FOV Center (along optical axis, at distance 0.5*perceptionRange from camera center)
    float mMedianDepth;

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity = false;

    //Transformation matrix between cameras in stereo fisheye
    Sophus::SE3<float> mTlr;
    Sophus::SE3<float> mTrl;

    // Imu bias
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints
    std::vector<MapPointPtr> mvpMapPoints;
    // For save relation without pointer, this is necessary for save/load function
    std::vector<long long int> mvBackupMapPointsId;

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
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFramePtr mpParent;
    std::set<KeyFramePtr> mspChildrens;
    std::set<KeyFramePtr> mspLoopEdges;
    std::set<KeyFramePtr> mspMergeEdges;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupParentId;
    std::vector<long unsigned int> mvBackupChildrensId;
    std::vector<long unsigned int> mvBackupLoopEdgesId;
    std::vector<long unsigned int> mvBackupMergeEdgesId;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap = nullptr;

    // Backup variables for inertial
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;

    // Backup for Cameras
    unsigned int mnBackupIdCamera, mnBackupIdCamera2;

    // Calibration
    //cv::Mat mK;  // exposed above 
    Eigen::Matrix3f mK_;

    // Mutex
    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexLineFeatures;
    std::mutex mMutexObjects;
    std::mutex mMutexMap;

public:
    GeometricCamera* mpCamera=nullptr, *mpCamera2=nullptr;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;
    std::vector<int> mvLeftToRightLinesMatch, mvRightToLeftLinesMatch;

    //Transformation matrix between cameras in stereo fisheye
    Sophus::SE3f GetRelativePoseTrl();
    Sophus::SE3f GetRelativePoseTlr();

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> mvKeysRight;
    const std::vector<cv::line_descriptor_c::KeyLine> mvKeyLinesRight;
    const std::vector<cv::line_descriptor_c::KeyLine> mvKeyLinesRightUn; // mvKeyLinesRightUn to be used only with fisheye cameras (i.e. pCamera2 != nullptr)

    const int NLeft, NRight;
    const int NlinesLeft=-1, NlinesRight=-1;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;
    std::vector< std::vector <std::vector<size_t> > > mLineGridRight;

    Sophus::SE3<float> GetRightPose();
    Sophus::SE3<float> GetRightPoseInverse();

    Eigen::Vector3f GetRightCameraCenter();
    Eigen::Matrix<float,3,3> GetRightRotation();
    Eigen::Vector3f GetRightTranslation();

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
    }

    static const std::vector<size_t> kEmptyVecSizet; 
};

} // namespace PLVS2

#endif // KEYFRAME_H
