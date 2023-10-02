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

namespace PLVS
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
    cv::Mat GetSE3() const; 
    
public: // friends    
    
    friend bool operator<(const ObjectObservation& l, const ObjectObservation& r);
    friend std::ostream &operator<<( std::ostream &out, const ObjectObservation &obs );  
    
public: 
    
    int nId; // id of the observed object 
    
    cv::Mat Tkc; // by default this is the identity, (optionally) observing Frame camera w.r.t. reference KeyFrame (see bFromK)
    cv::Mat Rco; // object w.r.t camera pc = Rco * po + tco
    cv::Mat tco; 
    
    float fScale;
    
    float fDistance; // distance camera-object 
    
    int nInliers; 
    float fReprojectionError; 
    
    int nMatchedMapPoints; 
    float fSim3Error; 
    
    bool bFromKF; // is this observation made from a KeyFrame?
                  // if the object is not detected on KeyFrame generation, the observation can be possible made in a previous frame (before actual KeyFrame generation)         
};


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
    
    void SetSim3Pose(const cv::Mat &Sow);    
    void SetSim3InversePose(const cv::Mat &Rwo, const cv::Mat &two, const double scale);        
        
    cv::Mat GetSim3Pose();
    cv::Mat GetSim3PoseInverse();
    cv::Mat GetObjectCenter();
    
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    double GetScale();
    
    cv::Mat GetInverseRotation();
    cv::Mat GetInverseTranslation();    
        
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
    cv::Mat mSowGBA;
    long unsigned int mnBAGlobalForKF;
    
    static std::mutex mGlobalMutex;      

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
            
    std::vector<cv::Point3f> mv3dRefPoints; // 3D reference points in object frame 
    std::vector<cv::Mat> mv3dRefCorners; // 3D reference corners in object frame    
    
    std::mutex mMutexImgCornersReprojected;
    std::vector<cv::Point2f> mvImCornersReprojected; // 3D reference corners reprojected in the current frame    
    
    std::mutex m3dCornersMutex;    
    std::vector<cv::Mat> mv3dCorners; // 3D corners in world frame     
            
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
    cv::Mat mSow; // Sow = [Row/s, tow; 0, 1]   ("object" frame contains mv3dRefPoints)
    cv::Mat mSwo; // Swo = [s*Rwo, two; 0, 1]   
    double mdScale; // scale (in mSow and mSwo)  from Object to World 
    cv::Mat mOw;  // center 

    bool mbBad;
    
    std::mutex mMutexPose;
         
    Map* mpMap;
         
    bool mbActive; 
};

} //namespace 

#endif // MAP_PLANAR_OBJECT_H
