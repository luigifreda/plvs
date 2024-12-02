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

#ifndef POINTCLOUD_MAPPING_H
#define POINTCLOUD_MAPPING_H 

#include <opencv2/core/core.hpp>

#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <deque>
#include <condition_variable>

#include "PointDefinitions.h"
#include "PointCloudMapTypes.h"

#include <pcl/common/transforms.h>

#include <boost/core/noncopyable.hpp>

#include "PointDefinitions.h"
#include "PointCloudKeyFrame.h"

namespace PLVS2
{
class KeyFrame;
class Map;
class Atlas; 
class LocalMapping; 
class Tracking; 

template<typename PointT>
class PointCloudMap;
class PointCloudCamParams;

template<typename PointT>
class PointCloudMapInput;

template<typename PointT>
class PointCloudAtlas;

struct Image4Viewer
{
    Image4Viewer():bReady(false) {}
    
    std::string name;
    bool bReady; 
    cv::Mat img; 
};

///	\class PointCloudMapping
///	\author Luigi Freda
///	\brief Main class for managing point cloud mapping
///	\note
///	\date
///	\warning
class PointCloudMapping: private boost::noncopyable
{
    
public:    
    static int skDownsampleStep;
    
    static const double kMaxDepthDistance; // [m]
    static const double kMinDepthDistance; // [m]
    static const int kNumKeyframesToQueueBeforeProcessing;
    static const int kMaxNumKeyFramesToInsertInMapInOneStep;
    static const int kTimeoutForWaitingKeyFramesMs; // [ms]    
    static const int kMainThreadSleepMs; // [ms]
    static const int kPerKeyFrameProcessSleepMs; // [ms]

    static const double kGridMapDefaultResolution;
    static const int kGridMapDefaultPointCounterThreshold;
    
    static const int kTimeoutForWaitingMapMs; // [ms]
    
    static const int kDepthFilterDiamater;  // diameter of the depth filter  
    static const double kDepthFilterSigmaDepth;  
    static const double kDepthSigmaSpace; 
    
    static const float kSementationMaxDepth; // [m]
    static const float kSegmentationMinFi; // dot product in [0,1]
    static const float kSegmentationMaxDelta; // [m] max allowed distance of two vertices on a convex surface 
    static const int kSegmentationSingleDepthMinComponentArea; 
    static const int kSegmenentationLabelConfidenceThreshold; 

public:

    typedef POINT_TYPE PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    
    using PointCloudMapType = PointCloudMapTypes::PointCloudMapType;
    
public:

    PointCloudMapping(const std::string &strSettingPath, Atlas* atlas, Tracking* tracking, LocalMapping* localMap);

    void InsertKeyFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyFrame);    

    void Shutdown();
    void Run();

    void Reset(); 

    void RebuildMap();
    
    void SaveMap();
    void LoadMap();
    
    bool IsFinished() const { return bFinished_; }
    
    void SetCameraCalibration(float fx, float fy, float cx, float cy); 

public: /// < getters 
    
    PointCloudT::Ptr GetMap();
    void GetMap(typename PointCloudT::Ptr& pCloud, typename PointCloudT::Ptr& pCloudUnstable, std::vector<unsigned int>& faces, bool copyUnstable = false );
    std::uint64_t GetMapTimestamp();
    
    PointCloudMapType GetMapType() const { return pPointCloudMapParameters_->pointCloudMapType; }
    
    std::vector<Image4Viewer> & GetVecImages() { return vecImages_; }
    
    int GetSegmentationLabelConfidenceThreshold() { return pPointCloudMapParameters_->segmentationLabelConfidenceThreshold; }
    
    std::shared_ptr<PointCloudAtlas<PointT> > GetPointCloudAtlas() { return mpPointCloudAtlas; }
    
protected:

    PointCloudT::Ptr GeneratePointCloudInCameraFrameBGRA(KeyFramePtr& kf, cv::Mat& color, cv::Mat& depth, cv::Mat& pixelToPointIndex, std::vector<unsigned int>& segmentsCardinality);
    PointCloudT::Ptr GeneratePointCloudInCameraFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyframe);
    PointCloudT::Ptr GeneratePointCloudInWorldFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyframe);

    bool UpdateMap();
    
    void UpdatePointCloudTimestamp();

    void InitCamGridPoints(const KeyFramePtr& kf, const cv::Size& depthSize);

    //void PrepareNewKeyFramesOld();
    void PrepareNewKeyFrames();    
    
    bool IntegratePointCloudKeyframe(const std::set<KeyFramePtr>& set_pKFs, PointCloudKeyFrame<PointT>::Ptr pcKeyframe);

    static void FilterDepthimage(cv::Mat &image,  const int diamater = 2*3+1, const double sigmaDepth = 0.02, const double sigmaSpace = 5);
    
protected:

    std::shared_ptr<std::thread> viewerThread_;

    bool bShutDown_ = false;
    std::mutex shutDownMutex_;

    /// < data to generate point clouds
    
    /* START: to be moved in PointCloudMap */
    // inserted pckeyframes 
    std::vector<PointCloudKeyFrame<PointT>::Ptr> pcKeyframes_;
    std::mutex pcKeyframesMutex_;
    std::atomic_bool bKeyframesAvailable_;
    std::atomic_bool bKeyframesToInsertInMap_;
    
    std::deque<PointCloudKeyFrame<PointT>::Ptr> pcKeyframesToReinsert_;
    std::mutex pcKeyframesToReinsertMutex_;
    
    // new incoming pckeyframes (to be init and processed)
    std::deque<PointCloudKeyFrame<PointT>::Ptr> pcKeyframesIn_;    
    std::mutex keyframesMutex_;
    std::condition_variable keyFramesCond_;
    uint16_t lastKeyframeIndex_ = 0;
    
    int baseKeyframeId_ = -1;
    /* END: to be moved in PointCloudMap */

    std::shared_ptr<PointCloudMap<PointT> > pPointCloudMap_;
    std::recursive_timed_mutex pointCloudMutex_;
    std::shared_ptr<PointCloudCamParams> pCameraParams_;

    std::uint64_t pointCloudTimestamp_;
    std::mutex pointCloudTimestampMutex_;

    std::shared_ptr<PointCloudMapParameters> pPointCloudMapParameters_;
    bool bIsNewSettings_;
    
    Atlas* mpAtlas = nullptr;
    //Map* mpMap;
    LocalMapping* mpLocalMapping = nullptr;
    Tracking* mpTracking = nullptr;
    std::shared_ptr<PointCloudAtlas<PointT> >  mpPointCloudAtlas;

    cv::Mat matCamGridPoints_;
    bool bInitCamGridPoints_;
    std::vector<std::vector<int> > vCamGridPointsNeighborsIdxs_;

    bool bActive_;
    
    size_t numKeyframesToQueueBeforeProcessing_; 
    
    std::vector<Image4Viewer> vecImages_;
    
    std::atomic_bool bFinished_;

    // loading/saving 
    
    bool mbLoadDensemap_;
    int mnSaveMapCount_;
    std::string msLoadFilename_; 
};


} //namespace PLVS2

#endif // POINTCLOUDMAPPING_H
