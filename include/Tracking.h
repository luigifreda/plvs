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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#ifdef USE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#endif

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "PointCloudMapping.h"

#include <mutex>


namespace PLVS
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class PointCloudMapping; 
class LineExtractor; 
class MapObject; 

class Tracking
{  
    static const int kNumMinFeaturesMonoInitialization;
    
    static const int kMaxNumOfKeyframesInLocalMap; 
    static const int kNumBestCovisibilityKeyFrames;    
    
public:
    static int skNumMinFeaturesStereoInitialization; 
    static int skNumMinFeaturesRGBDInitialization;    
    static float sknLineTrackWeigth; 
    static float skLineStereoMaxDist; 
    static float skMaxDistFovCenters; 
    
    static bool skUsePyramidPrecomputation; 

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetPointCloudMapping(std::shared_ptr<PointCloudMapping>& pPointCloudMapping);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal length
    void ChangeCalibration(const string &strSettingPath);
    
    void SetCalibration(const float fx, const float fy, const float cx, const float cy, const cv::Mat& DistCoef, const float bf);    

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    bool IsLineTracking() const {return mbLineTrackerOn; }
    bool IsObjectTracking() const {return mbObjectTrackerOn; }    

    cv::Mat& getMatK() { return mK; }
    cv::Mat& getMatDistCoef() { return mDistCoef; }
    
public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3,
        RELOCALIZE_IN_LOADED_MAP=4,
        NUM_TRACKING_STATES_1    // number of tracking stats minus 1
    };
static std::vector<std::string> vTrackingStateStrings; // must be kept in sync with eTrackingState
    const std::string& GetTrackingStateString() const { return vTrackingStateStrings[mState+1]; }

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImRGB;
    cv::Mat mImGray, mImgGrayRight;
    cv::Mat mImDepth;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFramePtr> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();
    
    void InitForRelocalizationInMap();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    void UpdateLastFrameLines(bool updatePose = false);
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalFeatures(); 
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();
    void SearchLocalLines();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();
      
    void PushKeyFrameInPointCloudMapping(KeyFramePtr& pKF);

    // In case of performing only localization, this flag is true when there are no matches to
    // points and lines in the map. Still tracking will continue if there are enough matches with temporal points or lines.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;
    
    //LSD
    bool mbLineTrackerOn = false;  // is line tracking active 
    std::shared_ptr<LineExtractor> mpLineExtractorLeft, mpLineExtractorRight; 
    
    //Object Tracking
    bool mbObjectTrackerOn = false;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFramePtr mpReferenceKF;
    std::vector<KeyFramePtr> mvpLocalKeyFrames;
    std::vector<MapPointPtr> mvpLocalMapPoints;
    std::vector<MapLinePtr> mvpLocalMapLines;
    std::vector<MapObjectPtr > mvpLocalMapObjects;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current point matches in frame
    int mnMatchesInliers;
    
    //Current line matches in frame
    int mnLineMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFramePtr mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPointPtr> mlpTemporalPoints; // collects temporary visual odometry points 
    list<MapLinePtr> mlpTemporalLines; // collects temporary visual odometry lines 
    
    std::shared_ptr<PointCloudMapping>  mpPointCloudMapping;
    
    bool mEnableDepthFilter;
    float mDepthCutoff;
#ifdef USE_CUDA
    cv::Ptr< cv::cuda::Filter > mpClosingFilter;
    cv::Ptr< cv::cuda::Filter > mpOpeningFilter;
#endif    
    
    bool mbUseFovCentersKfGenCriterion = false; 

};

} //namespace PLVS

#endif // TRACKING_H
