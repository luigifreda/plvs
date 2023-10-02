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

#ifndef MAP_PLANAR_OBJECT_H
#define MAP_PLANAR_OBJECT_H


#include "BoostArchiver.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>
#include <iostream>
#include <fstream>
#include <memory>

#include "Pointers.h"

#include "Eigen/Core"
#include "Thirdparty/Sophus/sophus/se3.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

namespace PLVS2
{

class KeyFrame;
class Map;
class Frame;
class MapPoint;


class ObjectObservation
{
public:
    
    ObjectObservation();
    
    // Copy constructor 
    ObjectObservation(const ObjectObservation &other);   
    // Copy assignment 
    ObjectObservation& operator=(const ObjectObservation& other);    
    
    // Get observation Sim3 transformation Tko (from Object to KeyFrame)
    Sophus::SE3f GetSE3() const; 
    
public: // friends    
    
    friend bool operator<(const ObjectObservation& l, const ObjectObservation& r);
    friend std::ostream &operator<<( std::ostream &out, const ObjectObservation &obs );  
    
public: 
    
    int nId; // id of the observed object 
    
    Sophus::SE3f Tkc; // by default this is the identity, (optionally) observing Frame camera w.r.t. reference KeyFrame (see bFromK)
    //cv::Mat Rco; // object w.r.t camera: pc = Rco * po + tco
    //cv::Mat tco; 
    Sophus::SE3f Tco;
    
    float fScale; // scale s in the Sim(3) transformation Swo = [s*Rwo, two; 0, 1]
    
    float fDistance; // distance camera-object 
    
    int nInliers; 
    float fReprojectionError; 
    
    int nMatchedMapPoints; 
    float fSim3Error; 
    
    bool bFromKF; // is this observation made from a KeyFrame?
                  // if the object is not detected on KeyFrame generation, the observation can be possible made in a previous frame (before actual KeyFrame generation)         
};


// TODO: add serialization? (objects could be moved)
class MapObject 
{
public: 
    
    static int skNumMinInliers;
    static float skMaxImgReprojectionError;    
    static float skMaxSim3Error;        
    static float skMatchRatio;
    
    static const float kObjectSize; 
    static const int kNumMinObvervationsForFiltering; 
    static const std::string kNameSaveLocFile; 
    
    static const float kFilteringQuadSize; 
    
public: 

    typedef std::shared_ptr<MapObject> Ptr;    
    typedef std::shared_ptr<const MapObject> ConstPtr;        
    
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapObject(Map* pMap, cv::Mat& imgObject, cv::Mat& K, cv::Mat& distCoeff, float matchRatio = skMatchRatio);
    
    void InitFeatures(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);
    void Detect(Frame* pFrame); 
    
    void SetActive(bool val) { mbActive = val; }
        
    bool IsOjectDetectedInCurrentFrame() const { return mbObjectDetectedInCurrenFrame; }
    bool IsOjectVisibleInCurrentFrame() const { return mbObjectVisibleInCurrenFrame; }
    bool IsLocalizedInCurrentFrame() const { return mbLocalizedInCurrentFrame; }
    bool IsScaleFixed() const { return mbScaleFixed; }
    bool IsLocalized() const { return mbLocalized; }    
    
    int GetNumInliers() const { return mCurrentObservation.nInliers; }
    int GetNumObservations() const { return nObs; }
    
    std::vector<cv::Point2f> GetCurrentImgCorners();
    std::vector<cv::Point2f> GetCurrentImgCornersReprojected();
    std::vector<cv::Point3f> GetCurrent3DCorners();
    
    ObjectObservation GetCurrentObjectObservation();
    
    void SetBadFlag();
    bool isBad();    
    
    bool AddObservation(const KeyFramePtr& pKF, const ObjectObservation& observation);
    void AddKeyFrameObservation(const KeyFramePtr& pKF);     
    void EraseObservation(const KeyFramePtr& pKF);
    std::map<KeyFramePtr,ObjectObservation> GetObservations();
    bool IsInKeyFrame(const KeyFramePtr& pKF);
    
    KeyFramePtr GetReferenceKeyFrame();
    void UpdateRefKeyFrame(const KeyFramePtr& pKF);
    
    void SaveRefObservations(); 
    
    void Update3DCorners(); 
    
    void SetSim3Pose(const Sophus::Sim3f &Sow);    
    void SetSim3InversePose(const Eigen::Matrix3f& Rwo, const Eigen::Vector3f& two, const double scale);        
        
    Sophus::Sim3f GetSim3Pose();
    Sophus::Sim3f GetSim3PoseInverse();
    Eigen::Vector3f GetObjectCenter();
    
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    double GetScale();
    
    Eigen::Matrix3f GetInverseRotation();
    Eigen::Vector3f GetInverseTranslation();    

    Map* GetMap();
    void UpdateMap(Map* pMap);
    
protected:

    void UndistortReferenceKeyPoints();  
    void InitImgCorners();    
    void Init3DRefPoints();
    
    void CheckMapPointsReplaced();
    
    void ApplyScaleTo3DRef(double scale);
    
    void Project3DCorners(Frame* pFrame);
    void FilterPoints(Frame* pFrame, std::vector<uchar>& frameMask);
    
public:

    long unsigned int mnId;
    static long unsigned int nNextId;    
    int nObs; // number of times the object has been observed     
    
    long int mnFirstFrame;
    long unsigned int mnTrackReferenceForFrame;    
    long unsigned int mnLastFrameSeen;
    
    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    Sophus::Sim3f mSowGBA;
    long unsigned int mnBAGlobalForKF;
    long unsigned int mnBALocalForMerge;
    
    static std::mutex mGlobalMutex;      

    // Variable used by merging
    Eigen::Matrix3f mRwoMerge;
    Eigen::Vector3f mtwoMerge;
    double mScaleMerge;
    
protected: 
    
    cv::Mat mImgRef;  // reference image of the object  
        
    std::vector<cv::KeyPoint> mvKeys, mvKeysUn; // keypoints in the reference image 
    cv::Mat mDescriptors; // descriptors in the reference image 
    
    std::vector<MapPointPtr> mvMPs; // map points corresponding to keypoints in the reference image    
    
    std::shared_ptr<cv::DescriptorMatcher> mpMatcher; 
    float mfNNmatchRatio;
    
    std::mutex mMutexImgCorners;
    std::vector<cv::Point2f> mvImgCornersRef; // corners in the reference image [top-left, top-right, bottom-right, bottom-left]
    std::vector<cv::Point2f> mvImCornersDetectedInCurrentFrame; // corners detected in the current frame
            
    std::vector<Eigen::Vector3f> mv3dRefPoints; // 3D reference points in object frame 
    std::vector<Eigen::Vector3f> mv3dRefCorners; // 3D reference corners in object frame    
    
    std::mutex mMutexImgCornersReprojected;
    std::vector<cv::Point2f> mvImCornersReprojected; // 3D reference corners reprojected in the current frame    
    
    std::mutex m3dCornersMutex;    
    std::vector<Eigen::Vector3f> mv3dCorners; // 3D corners in world frame     
            
    cv::Mat mK;
    cv::Mat mDistCoef;  
    
    std::mutex mMutexCurrentObservation; 
    ObjectObservation mCurrentObservation; 
    
    int mnLastNumMatchedPoints; 
    
    bool mbObjectDetectedInCurrenFrame; // detected ? 
    bool mbObjectVisibleInCurrenFrame;  // visible ?    
    bool mbLocalizedInCurrentFrame;     // true when detected and the estimated pose comes with an acceptable error
    bool mbScaleFixed; // object scale has been estimated? 
    bool mbLocalized; // is object localized?
        
    // Keyframes observing the object and associated observation 
    std::map<KeyFramePtr,ObjectObservation> mObservations;
    std::mutex mMutexObservations;     
    
    // Reference KeyFrame
    KeyFramePtr mpRefKF;
    
    // Sim3 object configuration 
    // NOTE 1: when transforming the object points: 
    //  1) first, we scale them by using s (or 1/s in the direction from "world" to "object")
    //  2) then, we roto-translate them by using [Rwo, two]  (or [Row, tow] where tow = -(Row* two)/s)
    // NOTE 2: here, we represent both transformation directions with a simple mapping p2 = scale*R*p1 + t  (where t possibly includes other scaling "effects")
    //         This allows to simplify and decouple the management of s,R,t in both the directions 
    Sophus::Sim3f mSow; // Sow = [Row/s, tow; 0, 1]   ("object" frame contains mv3dRefPoints)   NOTE: tow = -(Row* two)/s  (when comparing Sow with Swo)
    Sophus::Sim3f mSwo; // Swo = [s*Rwo, two; 0, 1]   
    double mdScale;     // scale s (used in the above mSow and mSwo); s is used as a multiplication factor in the direction from "object" to "world"
    Eigen::Vector3f mOw;// object center (two)
    
    bool mbBad;
    
    std::mutex mMutexPose;
    std::mutex mMutexMap;
     
    Map* mpMap;
         
    bool mbActive; 
};

} //namespace 

#endif // MAP_PLANAR_OBJECT_H
