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



#include <limits>
#include <unordered_map>
#include <utility>

//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <boost/thread/thread.hpp> 

#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "KeyFrame.h"
#include "Converter.h"

#include "PointCloudMapping.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "PointCloudMap.h"
#include "PointCloudAtlas.h"
#include "ColorOctomapServer.h"
#include "PointCloudMapFastFusion.h"
#include "PointCloudMapVoxblox.h"
#include "PointCloudMapChisel.h"
#include "PointCloudMapOctreePointCloud.h"
#include "PointCloudMapOctomap.h"
#include "PointCloudMapVoxelGridFilter.h"
#include "TimeUtils.h"
#include "Utils.h"  
#include "Stopwatch.h"
#include "PointUtils.h"
#include "PointCloudUtils.h"
#include "Neighborhood.h"


//#define PCL_VIEWER

#ifdef PCL_VIEWER
#include <pcl/visualization/cloud_viewer.h>
#endif


//#define _DEBUG // for activating debugging in opencv

namespace PLVS2
{

int PointCloudMapping::skDownsampleStep = 2;

const double PointCloudMapping::kMaxDepthDistance = 10; // [m]
const double PointCloudMapping::kMinDepthDistance = 0.01; // [m]

const int PointCloudMapping::kNumKeyframesToQueueBeforeProcessing = 5;//5;
const int PointCloudMapping::kMaxNumKeyFramesToInsertInMapInOneStep = 5;
const int PointCloudMapping::kTimeoutForWaitingKeyFramesMs = 5000; // [ms]    

const int PointCloudMapping::kMainThreadSleepMs = 10; // [ms]
const int PointCloudMapping::kPerKeyFrameProcessSleepMs = 5; // [ms]

const double PointCloudMapping::kGridMapDefaultResolution = 0.05;
const int PointCloudMapping::kGridMapDefaultPointCounterThreshold = 3;

const int PointCloudMapping::kTimeoutForWaitingMapMs = 5; // [ms]

const int PointCloudMapping::kDepthFilterDiamater = 2*3+1;  // diameter 
const double PointCloudMapping::kDepthFilterSigmaDepth = 0.02; 
const double PointCloudMapping::kDepthSigmaSpace = 5; 

const float PointCloudMapping::kSementationMaxDepth = 3; // [m]
const float PointCloudMapping::kSegmentationMinFi = 0.97; // dot product in [0,1]
const float PointCloudMapping::kSegmentationMaxDelta = 0.02; // [m] max allowed distance of two vertices on a convex surface 
const int PointCloudMapping::kSegmentationSingleDepthMinComponentArea = 20; // default  
const int PointCloudMapping::kSegmenentationLabelConfidenceThreshold = 5; // default


//typedef FourNeighborhoodIndices NeighborhoodT; 
//typedef EigthNeighborhoodIndices NeighborhoodT;
typedef EigthNeighborhoodIndicesFast NeighborhoodT;



PointCloudMapping::PointCloudMapping(const string &strSettingPath, Atlas* atlas, Tracking* tracking, LocalMapping* localMap): 
mpAtlas(atlas), mpTracking(tracking), mpLocalMapping(localMap), bInitCamGridPoints_(false),bFinished_(true),mnSaveMapCount_(0)
{
    cv::FileStorage fsSettings(strSettingPath, cv::FileStorage::READ);
    
    auto nodeVersion = fsSettings["File.version"];
    bIsNewSettings_ = (!nodeVersion.empty() && nodeVersion.isString() && nodeVersion.string() == "1.0");

    // < StereoDense
    std::cout << std::endl  << "StereoDense Parameters: " << std::endl; 
    std::string stereoDenseStringType = Utils::GetParam(fsSettings, "StereoDense.type", std::string("libelas")); 
    if (stereoDenseStringType == "libelas") PointCloudKeyFrame<PointT>::skStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibelas;
    if (stereoDenseStringType == "libsgm") PointCloudKeyFrame<PointT>::skStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibsgm;  
    if (stereoDenseStringType == "opencv") PointCloudKeyFrame<PointT>::skStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCV; 
    if (stereoDenseStringType == "opencvcuda") PointCloudKeyFrame<PointT>::skStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCVCuda;  
    
    PointCloudKeyFrame<PointT>::skbNeedRectification = Utils::GetParam(fsSettings, "StereoDense.needRectification", false);
    if(PointCloudKeyFrame<PointT>::skbNeedRectification){
        std::cout << "PointCloudMapping need rectification" << std::endl;
        // check we are using KB cameras 
        const auto& settings = Settings::instance();
        MSG_ASSERT(settings,"Settings not initialized, you must use the new examples in order to init Settings!");
        MSG_ASSERT(settings->cameraType() == Settings::CameraType::KannalaBrandt,"Only KB cameras are supported for now!");

        settings->precomputeRectificationMaps(false /*update calibration*/);
    }
    

    // < PointCloudMapping
    std::cout << std::endl  << "PointCloudMapping Parameters: " << std::endl;   
    bActive_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.on", 0)) != 0;
    
    // read the desired map model 
    std::string pointCloudMapTypeStringDefault = PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kOctreePoint];
    std::string pointCloudMapStringType = Utils::GetParam(fsSettings, "PointCloudMapping.type", pointCloudMapTypeStringDefault);
    std::cout << "point cloud map type: " << pointCloudMapStringType << std::endl;
    
    numKeyframesToQueueBeforeProcessing_ = Utils::GetParam(fsSettings, "PointCloudMapping.numKeyframesToQueueBeforeProcessing", kNumKeyframesToQueueBeforeProcessing);
    
    skDownsampleStep = Utils::GetParam(fsSettings, "PointCloudMapping.downSampleStep", 2);
    
    double resolution = Utils::GetParam(fsSettings, "PointCloudMapping.resolution", kGridMapDefaultResolution);
    
    std::cout << "PointCloudMapping::PointCloudMapping() - resolution: " <<  resolution << std::endl;

    const double maxDepthDistance = Utils::GetParam(fsSettings, "PointCloudMapping.maxDepth", kMaxDepthDistance);
    const double minDepthDistance = Utils::GetParam(fsSettings, "PointCloudMapping.minDepth", kMinDepthDistance);
    
    KeyFrame::skFovCenterDistance = 0.5*(minDepthDistance + maxDepthDistance);
    
    double imageDepthScale = Utils::GetParam(fsSettings, "DepthMapFactor", 1.);

    // read parameters for specific map models 
    bool bUseCarving = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.useCarving", 1)) != 0;
    
    bool bRemoveUnstablePoints = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.removeUnstablePoints", 1)) != 0;
    
    bool bResetOnSparseMapChange = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.resetOnSparseMapChange", 1)) != 0;
    bool bCloudDeformationOnSparseMapChange = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.cloudDeformationOnSparseMapChange", 0)) != 0;
    if(bCloudDeformationOnSparseMapChange) bResetOnSparseMapChange = false;
    
    int nPointCounterThreshold = Utils::GetParam(fsSettings, "PointCloudMapping.pointCounterThreshold", kGridMapDefaultPointCounterThreshold);

    PointCloudMapVoxblox<PointT>::skIntegrationMethod = Utils::GetParam(fsSettings, "PointCloudMapping.voxbloxIntegrationMethod", PointCloudMapVoxblox<PointT>::skIntegrationMethod);

    /// < NOTE: here we manage a simple model (without distortion) which is used for projecting point clouds;
    /// <       it assumes input rectified images; in particular chisel framework works under these assumptions
    pCameraParams_ = std::make_shared<PointCloudCamParams>();
    pCameraParams_->minDist = minDepthDistance;
    pCameraParams_->maxDist = maxDepthDistance;
    // Let's reuse the parsing we have in Tracking class 
    pCameraParams_->mK = mpTracking->GetMatK().clone();
    pCameraParams_->mDistCoef = mpTracking->GetMatDistCoef().clone();
    pCameraParams_->fx = pCameraParams_->mK.at<float>(0,0);
    pCameraParams_->fy = pCameraParams_->mK.at<float>(1,1);
    pCameraParams_->cx = pCameraParams_->mK.at<float>(0,2);
    pCameraParams_->cy = pCameraParams_->mK.at<float>(1,2);
    pCameraParams_->bf= mpTracking->GetBf();
    const float imageScale = mpTracking->GetImageScale();

    if(!bIsNewSettings_)
    {
        pCameraParams_->width = fsSettings["Camera.width"];
        pCameraParams_->height = fsSettings["Camera.height"];
        if(imageScale != 1.f)
        {
            pCameraParams_->width  *= imageScale;
            pCameraParams_->height *= imageScale;
        }
    }
    else 
    {
        const Settings* settings = Settings::instance();
        const auto camera1 = settings->camera1();
        const auto imageSize = settings->newImSize();
        pCameraParams_->width = imageSize.width;
        pCameraParams_->height = imageSize.height;

        if(PointCloudKeyFrame<PointT>::skbNeedRectification){
            // update point cloud camera calibration parameters
            const cv::Mat& P1Rect = settings->P1Rect();
            pCameraParams_->fx = P1Rect.at<double>(0,0);
            pCameraParams_->fy = P1Rect.at<double>(1,1);
            pCameraParams_->cx = P1Rect.at<double>(0,2);
            pCameraParams_->cy = P1Rect.at<double>(1,2);
            pCameraParams_->bf = settings->b() * P1Rect.at<double>(0,0);     

            pCameraParams_->mDistCoef = cv::Mat::zeros(4,1,CV_32F);
            pCameraParams_->mK = cv::Mat::eye(3,3,CV_32F);
            pCameraParams_->mK.at<float>(0,0) = pCameraParams_->fx;
            pCameraParams_->mK.at<float>(1,1) = pCameraParams_->fy;
            pCameraParams_->mK.at<float>(0,2) = pCameraParams_->cx;
            pCameraParams_->mK.at<float>(1,2) = pCameraParams_->cy;            
        }
    }

    
    pointCloudTimestamp_ = 0;

    bKeyframesAvailable_ = false;
    bKeyframesToInsertInMap_ = false;
    
    vecImages_.resize(5);

    bool bFilterDepthImages = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.on", 0)) != 0;
    int depthFilterDiameter = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.diameter", kDepthFilterDiamater);  
    double depthFilterSigmaDepth = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.sigmaDepth", kDepthFilterSigmaDepth); 
    double depthSigmaSpace = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.sigmaSpace", kDepthSigmaSpace); 
    
    mbLoadDensemap_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.loadMap", 0)) != 0;
    msLoadFilename_ = Utils::GetParam(fsSettings, "PointCloudMapping.loadFilename", std::string()); 
    
    cout << endl  << "Segmentation Parameters: " << endl; 
    bool bSegmentationOn = static_cast<int> (Utils::GetParam(fsSettings, "Segmentation.on", 0)) != 0;
    
    // < disable segmentation if you are not using octreepoint
    //if( (bSegmentationOn) && (pPointCloudMapParameters_->pointCloudMapType != PointCloudMapTypes::kOctreePoint) )
    if( (bSegmentationOn) && (pointCloudMapStringType != PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kOctreePoint]) )    
    {
        cout << endl  << "!!!WARNING: segmentation disabled without octree_point!!!" << endl; 
        bSegmentationOn = false; 
    }
    
    if(bSegmentationOn)
    {
        std::cout << "WARNING: forcing numKeyframesToQueueBeforeProcessing to 1" << std::endl; 
        numKeyframesToQueueBeforeProcessing_ = 1; 
        
        std::cout << "WARNING: forcing filter depth on" << std::endl; 
        bFilterDepthImages = true; 
    }    
    bool bSegmentationErosionDilationOn = static_cast<int> (Utils::GetParam(fsSettings, "Segmentation.erosionDilationOn", 1)) != 0;
    
    float sementationMaxDepth = Utils::GetParam(fsSettings, "Segmentation.maxDepth", kSementationMaxDepth);
    float segmentationMinFi = Utils::GetParam(fsSettings, "Segmentation.minFi", kSegmentationMinFi);
    float segmentationMaxDelta = Utils::GetParam(fsSettings, "Segmentation.maxDelta", kSegmentationMaxDelta);
    int segmentationSingleDepthMinComponentArea = Utils::GetParam(fsSettings, "Segmentation.singleDepth.minComponentArea", kSegmentationSingleDepthMinComponentArea);
    int segmentationLineDrawThinckness = Utils::GetParam(fsSettings, "Segmentation.lineDrawThinckness", 2);
    
    int segmentationLabelConfidenceThreshold = Utils::GetParam(fsSettings, "Segmentation.labelConfidenceThreshold", kSegmenentationLabelConfidenceThreshold);
   
    float segmentationMaxAngleForNormalAssociation = Utils::GetParam(fsSettings, "Segmentation.maxAngleForNormalAssociation", 20);
    LabelMap::skLabelsMatchingMinOverlapPerc = Utils::GetParam(fsSettings, "Segmentation.labelsMatchingMinOverlapPerc", LabelMap::kLabelsMatchingMinOverlaPercDefault);
    LabelMap::skLabelsMatchingMinOverlapPoints = Utils::GetParam(fsSettings, "Segmentation.labelsMatchingMinOverlapPoints", LabelMap::kLabelsMatchingMinOverlapPointsDefault);
    GlobalLabelMap::skLabelsMatchingMinOverlapPerc = Utils::GetParam(fsSettings, "Segmentation.globalLabelsMatchingMinOverlapPerc", GlobalLabelMap::kLabelsMatchingMinOverlaPercDefault);
    
    /// < Fill in the point cloud map params 
    
    pPointCloudMapParameters_ = std::make_shared<PointCloudMapParameters>();
    pPointCloudMapParameters_->pointCloudMapStringType = pointCloudMapStringType;
    //pPointCloudMapParameters_->pointCloudMapType  // this is set in PointCloudAtlas<PointT>::NewPointCloudMap()
    pPointCloudMapParameters_->resolution = resolution;
    
    pPointCloudMapParameters_->minDepthDistance = minDepthDistance;    
    pPointCloudMapParameters_->maxDepthDistance = maxDepthDistance;
    pPointCloudMapParameters_->maxDepthDistance = maxDepthDistance;   
    pPointCloudMapParameters_->imageDepthScale = imageDepthScale;
    
    pPointCloudMapParameters_->bUseCarving = bUseCarving;
    
    pPointCloudMapParameters_->nPointCounterThreshold = nPointCounterThreshold;   
    pPointCloudMapParameters_->bRemoveUnstablePoints = bRemoveUnstablePoints;
    pPointCloudMapParameters_->bResetOnSparseMapChange = bResetOnSparseMapChange;
    
    pPointCloudMapParameters_->bCloudDeformationOnSparseMapChange = bCloudDeformationOnSparseMapChange;
    
    pPointCloudMapParameters_->bFilterDepthImages = bFilterDepthImages;
    pPointCloudMapParameters_->depthFilterDiameter = depthFilterDiameter;  // diameter of the depth filter  
    pPointCloudMapParameters_->depthFilterSigmaDepth = depthFilterSigmaDepth;  
    pPointCloudMapParameters_->depthSigmaSpace = depthSigmaSpace;     
     
    pPointCloudMapParameters_->bSegmentationOn = bSegmentationOn;
    pPointCloudMapParameters_->sementationMaxDepth = sementationMaxDepth;  // [m]
    pPointCloudMapParameters_->segmentationMinFi = segmentationMinFi;    // dot product in [0,1]
    pPointCloudMapParameters_->segmentationMaxDelta = segmentationMaxDelta; // [m] max allowed distance of two vertices on a convex surface 
    pPointCloudMapParameters_->segmentationSingleDepthMinComponentArea = segmentationSingleDepthMinComponentArea;
    pPointCloudMapParameters_->segmentationLineDrawThinckness = segmentationLineDrawThinckness;
    pPointCloudMapParameters_->segmentationMaxAngleForNormalAssociation = segmentationMaxAngleForNormalAssociation;
    pPointCloudMapParameters_->segmentationMinCosForNormalAssociation = cos(segmentationMaxAngleForNormalAssociation*M_PI/180.);
        
    pPointCloudMapParameters_->segmentationLabelConfidenceThreshold = segmentationLabelConfidenceThreshold;
    pPointCloudMapParameters_->bSegmentationErosionDilationOn = bSegmentationErosionDilationOn;      
    
    pPointCloudMapParameters_->nDownsampleStep = skDownsampleStep;
    
    
    /// < Create and set the first point cloud map 
    mpPointCloudAtlas = make_shared<PointCloudAtlas<PointT> >(atlas, pPointCloudMapParameters_);    
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);  
    pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();    
    //NOTE: each new pPointCloudMap will be set with pPointCloudMapParameters_ by mpPointCloudAtlas
    
    /// < NOTE: we assume frames are already rectified and depth camera and rgb camera have the same projection models 
    pPointCloudMap_->SetColorCameraModel(*pCameraParams_);
    pPointCloudMap_->SetDepthCameraModel(*pCameraParams_);
    
    //TODO: Luigi these can be removed since now we should use pPointCloudMapParameters_
//    pPointCloudMap_->SetDownsampleStep(skDownsampleStep);
//    pPointCloudMap_->SetPerformCarving(bUseCarving);
//    pPointCloudMap_->SetRemoveUnstablePoints(bRemoveUnstablePoints);
//    pPointCloudMap_->SetResetOnMapChange(bResetOnSparseMapChange);
//    pPointCloudMap_->SetKfAdjustmentOnMapChange(bCloudDeformationOnSparseMapChange);
//    pPointCloudMap_->SetPerformSegmentation(bSegmentationOn);
//    pPointCloudMap_->SetMinCosForNormalAssociation(cos(segmentationMaxAngleForNormalAssociation*M_PI/180.));
        
    // load dense map 
    if( !msLoadFilename_.empty() &&  mbLoadDensemap_ )
    {
        pPointCloudMap_->LoadMap(msLoadFilename_);
    }
    
    /// < last, launch the thread
    if(bActive_)
    {
        viewerThread_ = make_shared<thread>(bind(&PointCloudMapping::Run, this));
    }
    
}

void PointCloudMapping::Shutdown() 
{
    if(!bActive_) return; 
    
    {
        unique_lock<mutex> lck_shutdown(shutDownMutex_);
        bShutDown_ = true;

        unique_lock<mutex> lck_keyframes(keyframesMutex_);
        keyFramesCond_.notify_one();
    }
    viewerThread_->join();
}

//void PointCloudMapping::InsertKeyFrameOld(KeyFramePtr kf, cv::Mat& color, cv::Mat& depth)
//{
//    if (!bActive_) return;
//
//    cout << "receive a keyframe, id = " << kf->mnId << endl;
//
//    unique_lock<mutex> lck(keyframesMutex_);
//
//    // N.B.: no memory sharing for color and depth
//    keyframes_.push_back(kf);
//    colorImgs_.push_back(color.clone());
//    depthImgs_.push_back(depth.clone());
//
//    bKeyframesAvailable_ = true;
//
//    if (keyframes_.size() >= numKeyframesToQueueBeforeProcessing_)
//    {
//        keyFramesCond_.notify_one();
//    }
//}

#define INIT_PCKF_ON_INSERT 0
void PointCloudMapping::InsertKeyFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyFrame)
{
    if (!bActive_) return;

    unique_lock<mutex> lck(keyframesMutex_);

    cout << "Received a keyframe, id = " << pcKeyFrame->pKF->mnId << " (# queued: " << pcKeyframesIn_.size() << ")" << endl;
    if(baseKeyframeId_< 0) baseKeyframeId_ = pcKeyFrame->pKF->mnId; 
     
#if INIT_PCKF_ON_INSERT    
    pcKeyFrame->Init(pCameraParams_.get()); // N.B.: no memory sharing for color and depth  
#endif 
    
    pcKeyframesIn_.push_back(pcKeyFrame);

    bKeyframesAvailable_ = true;

    if (pcKeyframesIn_.size() >= numKeyframesToQueueBeforeProcessing_)
    {
        keyFramesCond_.notify_one();
    }    
}

//void PointCloudMapping::PrepareNewKeyFramesOld()
//{
//    unique_lock<mutex> lck_pcKeyframes(pcKeyframesMutex_);
//
//    unique_lock<mutex> lck(keyframesMutex_);
//
//    for (size_t i = 0; i < keyframes_.size(); i++)
//    {
//        KeyFramePtr kf = keyframes_[i];
//        cv::Mat& color = colorImgs_[i];
//        cv::Mat& depth = depthImgs_[i];
//
//        cout << "preparing keyframe, id = " << kf->mnId << endl;
//        PointCloudKeyFrame<PointT>::Ptr pcKeyframe(new PointCloudKeyFrame<PointT>(kf, color, depth)); 
//        pcKeyframes_.push_back(pcKeyframe);
//    }
//    keyframes_.clear();
//    colorImgs_.clear();
//    depthImgs_.clear();
//
//    bKeyframesAvailable_ = false;
//}

void PointCloudMapping::PrepareNewKeyFrames()
{
    unique_lock<mutex> lck_pcKeyframes(pcKeyframesMutex_);

    unique_lock<mutex> lck(keyframesMutex_);
    
    if(pcKeyframesIn_.empty()) return; 
    
    if(mpAtlas->isInertial())
    {
        if(!mpAtlas->isImuInitialized()) return; 
    }

    for (auto it=pcKeyframesIn_.begin(); it != pcKeyframesIn_.end(); )
    {
        PointCloudKeyFrame<PointT>::Ptr pcKeyframe = *it;
        cout << "Preparing keyframe, id = " << pcKeyframe->pKF->mnId << " (Bad: " << (int)pcKeyframe->pKF->isBad() << ", LBA count: " << pcKeyframe->pKF->mnLBACount <<  ") (#queued: " << pcKeyframesIn_.size() << ")" << endl;
        
#if !INIT_PCKF_ON_INSERT          
        pcKeyframe->Init(pCameraParams_.get()); // N.B.: no memory sharing for color and depth  
#endif 
        
        pcKeyframe->PreProcess();
        
        //NOTE: insert only once LBA has adjusted the KF a first time
        if(pcKeyframe->pKF->mnLBACount>0 || pcKeyframe->pKF->mbFixed)
        {
            pcKeyframes_.push_back(pcKeyframe);
            it = pcKeyframesIn_.erase(it);
        }
        else
        {
            it++;
        }
        
    }
    //pcKeyframesIn_.clear();

    bKeyframesAvailable_ = false;
}

void PointCloudMapping::Run()
{
    bFinished_ = false; 

#ifdef PCL_VIEWER
    pcl::visualization::CloudViewer viewer("viewer");
    while (!viewer.wasStopped())
#else
    while (1)
#endif
    {
        {
            unique_lock<mutex> lck_shutdown(shutDownMutex_);
            if (bShutDown_)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframes(keyframesMutex_);
            if ((!bKeyframesAvailable_) && (!bKeyframesToInsertInMap_))
            {
                //keyFramesCond_.wait(lck_keyframes); 
                keyFramesCond_.wait_for(lck_keyframes, std::chrono::milliseconds(kTimeoutForWaitingKeyFramesMs));  
                
            }
        }

        PrepareNewKeyFrames();
        UpdateMap();
        
        SENDALLCLOUD;

#ifdef PCL_VIEWER        
        {
            unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);
            pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
            viewer.showCloud(pPointCloudMap_->GetMap());
        }
#endif
        
        if (kMainThreadSleepMs > 0)
        {
            //boost::this_thread::sleep(boost::posix_time::milliseconds(kMainThreadSleepMs));
            std::this_thread::sleep_for(std::chrono::milliseconds(kMainThreadSleepMs));
        }

    }
    
    bFinished_ = true; 
}

bool PointCloudMapping::UpdateMap()
{
    int num_keyframes_inserted_in_map = 0;
    bool b_have_time_to_reinsert_old_frames = true; 

    // get all the new keyframes!
    Map* map = mpAtlas->GetCurrentMap();
    set<KeyFramePtr> set_pKFs = map->GetSetKeyFrames();
    mpLocalMapping->AddNewKeyFramesToSet(set_pKFs);
    

    unique_lock<recursive_timed_mutex> lck_globalMap(pointCloudMutex_);
    unique_lock<mutex> lck_pcKeyframes(pcKeyframesMutex_);
    
    pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();

    size_t i, iEnd;
        
    /// < integrate most recent keyframes 
    for (i = lastKeyframeIndex_, iEnd=pcKeyframes_.size(); i < iEnd; i++)
    {
        PointCloudKeyFrame<PointT>::Ptr pcKeyframe = pcKeyframes_[i];

        if(IntegratePointCloudKeyframe(set_pKFs, pcKeyframe))
        {
            num_keyframes_inserted_in_map++;
        }

        if (kPerKeyFrameProcessSleepMs > 0)
        {
            //boost::this_thread::sleep(boost::posix_time::milliseconds(kPerFrameProcessSleepMs));
            std::this_thread::sleep_for(std::chrono::milliseconds(kPerKeyFrameProcessSleepMs));
        }

        if (num_keyframes_inserted_in_map >= kMaxNumKeyFramesToInsertInMapInOneStep) 
        {
            b_have_time_to_reinsert_old_frames = false;
            break; /// < STOP loop
        }
    }

    bKeyframesToInsertInMap_ = i < pcKeyframes_.size();
    //lastKeyframeSize = pcKeyframes.size();
    lastKeyframeIndex_ = std::max(0, (int) i - 1);
    std::cout << "lastKeyframeIndex: " << lastKeyframeIndex_ << std::endl; 
    
    
    /// < integrate old keyframes which must be reintegrated 
    if(b_have_time_to_reinsert_old_frames)
    {
        while(!pcKeyframesToReinsert_.empty())
        {
            PointCloudKeyFrame<PointT>::Ptr pcKeyframe = pcKeyframesToReinsert_.back();
            pcKeyframesToReinsert_.pop_back();

            if(IntegratePointCloudKeyframe(set_pKFs, pcKeyframe))
            {
                num_keyframes_inserted_in_map++;
            }

            if (kPerKeyFrameProcessSleepMs > 0)
            {
                //boost::this_thread::sleep(boost::posix_time::milliseconds(kPerFrameProcessSleepMs));
                std::this_thread::sleep_for(std::chrono::milliseconds(kPerKeyFrameProcessSleepMs));
                
            }

            if (num_keyframes_inserted_in_map >= kMaxNumKeyFramesToInsertInMapInOneStep) 
            {
                break; /// < STOP loop
            }
        }    
    }
    bKeyframesToInsertInMap_ = bKeyframesToInsertInMap_ || (!pcKeyframesToReinsert_.empty());
    
    
    if (num_keyframes_inserted_in_map>0)
    {
        TICKCLOUD("PC::UpdateMap");
        int size_new_map = pPointCloudMap_->UpdateMap();
        TOCKCLOUD("PC::UpdateMap");
        
        UpdatePointCloudTimestamp();

        cout << "updated global map, size=" << size_new_map << endl;
    }

    return (num_keyframes_inserted_in_map>0);
}

void PointCloudMapping::UpdatePointCloudTimestamp()
{
    unique_lock<recursive_timed_mutex> lck_globalMap(pointCloudMutex_);
    pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();    
    std::uint64_t timestamp = pPointCloudMap_->GetMapTimestamp();

    unique_lock<mutex> lck(pointCloudTimestampMutex_); //shared with drawing thread 
    pointCloudTimestamp_ = timestamp;    
}

bool PointCloudMapping::IntegratePointCloudKeyframe(const std::set<KeyFramePtr>& set_pKFs, PointCloudKeyFrame<PointT>::Ptr pcKeyframe)
{    
    if(!pcKeyframe) return false; 
    
    bool b_integrated = false;
        
    // check if the keyframe is in the std::map 
    if ((pcKeyframe->bIsValid) && (set_pKFs.count(pcKeyframe->pKF) > 0))
    {
        if (!pcKeyframe->bInMap)
        {
            cout << "Integrating point cloud of KF " << pcKeyframe->pKF->mnId << "/" << (pcKeyframes_.size()-1) + baseKeyframeId_ << " (Map: "<<  pcKeyframe->pKF->GetMap()->GetId() << ")" << endl;
   
            PointCloudT::Ptr pCloudCamera = GeneratePointCloudInCameraFrame(pcKeyframe);  
            
            std::shared_ptr<PointCloudMapInput<PointT> > pPointCloudMapInput = std::make_shared<PointCloudMapInput<PointT> >(
                    pcKeyframe, pPointCloudMapParameters_->minDepthDistance, pPointCloudMapParameters_->maxDepthDistance, 
                    pPointCloudMapParameters_->imageDepthScale, pCloudCamera->header.stamp);
                    
            switch (pPointCloudMapParameters_->pointCloudMapType) 
            {
            case PointCloudMapTypes::kChisel:            
                // two possible modes   
                if(pPointCloudMapParameters_->bUseCarving)
                    pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloudAndDepthImage;
                else
                    pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloud;                    
                //pPointCloudMapInput->type = PointCloudMapInput<PointT>::kColorAndDepthImages;  /// < NOTE: much sloweeer that inserting with pointcloud!!!!
                break;
                
            case PointCloudMapTypes::kFastFusion:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kColorAndDepthImages;
                break;
                   
            case PointCloudMapTypes::kOctreePoint:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloudAndDepthImage;
                break;

            default:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloud;
            }
            
            TICKCLOUD("PC::InsertData");
            
            // check the correspondence between entry KF and current point cloud map
            if(pcKeyframe->pKF->GetMap() == pPointCloudMap_->GetCorrespodingSparseMap())
            {
                pPointCloudMap_->InsertData(pPointCloudMapInput);
            }
#if 1           
            else
            {
                std::cout << "WARNING: integrating the point cloud for KF in different map!" << std::endl; 
                auto pPointCloudMapOld = mpPointCloudAtlas->GetCorrespondingPointCloudMap(pcKeyframe->pKF->GetMap());
                if(!pPointCloudMapOld->IsBad())
                    pPointCloudMapOld->InsertData(pPointCloudMapInput);                
            }
#endif 
            
            TOCKCLOUD("PC::InsertData");
                    
            //  release useless data
            switch(pPointCloudMapInput->type)
            {
            case PointCloudMapInput<PointT>::kPointCloud:
                pcKeyframe->ReleaseColor();
                pcKeyframe->ReleaseDepth();  
                break;
                
            case PointCloudMapInput<PointT>::kDepthImage:
            case PointCloudMapInput<PointT>::kPointCloudAndDepthImage:
                pcKeyframe->ReleaseColor(); 
                break;    
                
            case PointCloudMapInput<PointT>::kColorAndDepthImages:
                ; // nop
                break;    
                
            default:
                ;//nop
            }
            
            //if(bKfAdjustmentOnSparseMapChange_) pcKeyframe->Release();

            pcKeyframe->bInMap = true;

            b_integrated = true;
        }
    }
    else
    {
        cout << "discarding point cloud for kf " << pcKeyframe->pKF->mnId << "/" << pcKeyframes_.size() << " ********************* " << endl;
        //cout << "reason -  valid: " << pcKeyframe->bIsValid << ", in keyframes: " << set_pKFs.count(pcKeyframe->pKF) << std::endl;
        // get rid of unused keyframes 
        if (pcKeyframe->bIsValid) pcKeyframe->Clear();
    }
    
    return b_integrated;
}

void PointCloudMapping::Reset()
{
    std::cout << "PointCloudMapping::Reset() - start " << std::endl;

    
    unique_lock<recursive_timed_mutex> lckPc(pointCloudMutex_); // NOTE: this is needed in order to sync w.r.t. other point cloud processing ops

    /// < NOTE: the following line is commented since Atlas::clearAtlas() is directly connected to PointCloudAtlas::clearAtlas()
    //mpPointCloudAtlas->clearAtlas();

    /// < NOTE: the following two lines are commented since Atlas::clearMap() is directly connected to PointCloudAtlas::clearMap()
    //pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
    //pPointCloudMap_->Clear();
    
    unique_lock<mutex> lckKFs(keyframesMutex_); // block the insertion of new frames
    bKeyframesAvailable_ = false;
    bKeyframesToInsertInMap_ = false; 
    pcKeyframesIn_.clear();
    
    
    unique_lock<mutex> lckPcKFs(pcKeyframesMutex_);
    pcKeyframes_.clear();
    lastKeyframeIndex_ = 0;
    
    pcKeyframesToReinsert_.clear();

    std::cout << "PointCloudMapping::Reset() - end" << std::endl;    
}

void PointCloudMapping::RebuildMap()
{
    if(!bActive_) return; 
    
    std::cout << "***********************************************" << std::endl;
    std::cout << "PointCloudMapping::RebuildMap() ***************" << std::endl;
    std::cout << "***********************************************" << std::endl;

    unique_lock<mutex> lck(keyframesMutex_); // block the insertion of new frames

    {
        unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);
        pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
        //pPointCloudMap_->Clear();
        pPointCloudMap_->OnMapChange();
    }

    if(!pPointCloudMapParameters_->bCloudDeformationOnSparseMapChange)
    {
        unique_lock<mutex> lck(pcKeyframesMutex_);
#if 0        
        for (size_t i = 0, iEnd=pcKeyframes_.size(); i < iEnd; i++)
        {
            pcKeyframes_[i]->bInMap = false;
        }
        lastKeyframeIndex_ = 0;
#else
        pcKeyframesToReinsert_.clear();
        for (size_t i = 0; i < lastKeyframeIndex_; i++)
        //for (size_t i = std::max(lastKeyframeSize-1,0); i >=0 ; i--)
        {
            if(pcKeyframes_[i]->bIsValid)
            {
                pcKeyframes_[i]->bInMap = false;
                pcKeyframesToReinsert_.push_front(pcKeyframes_[i]);
            }
        }
#endif

        bKeyframesToInsertInMap_ = true;        
        keyFramesCond_.notify_one();        
    }
    
    UpdatePointCloudTimestamp();
    
}

/// < here we assume the camera calibration matrix is fixed (all the keyframes must share the same kf->mK, kf->mDistCoef)
// this is just computed once!
void PointCloudMapping::InitCamGridPoints(const KeyFramePtr& kf, const cv::Size& depthSize)
{
    if (bInitCamGridPoints_) return;
    bInitCamGridPoints_ = true;

    auto checkFloatEqual = [](float a, float b) -> bool { return fabs(a - b) < 1e-3; };

    const double fx = pCameraParams_->fx;
    const double fy = pCameraParams_->fy;
    const double cx = pCameraParams_->cx;
    const double cy = pCameraParams_->cy;
    const double bf = pCameraParams_->bf;
    const cv::Mat& K = pCameraParams_->mK;
    const cv::Mat& distCoef = pCameraParams_->mDistCoef;

#if 1
    if(!PointCloudKeyFrame<PointT>::skbNeedRectification){
        MSG_ASSERT( checkFloatEqual(kf->fx,pCameraParams_->fx) && checkFloatEqual(kf->fy,pCameraParams_->fy) && 
                    checkFloatEqual(kf->cx,pCameraParams_->cx) && checkFloatEqual(kf->cy,pCameraParams_->cy) && 
                    checkFloatEqual(kf->mbf,pCameraParams_->bf), 
                    "Camera parameters are not the same for all the keyframes \n" 
                    << "kf params: " << kf->fx << " " << kf->fy << " " << kf->cx << " " << kf->cy << ", bf: " << kf->mbf << "\n" 
                    << "cam params: " << pCameraParams_->fx << " " << pCameraParams_->fy << " " << pCameraParams_->cx << " " << pCameraParams_->cy << ", bf: " << pCameraParams_->bf);
        MSG_ASSERT( checkFloatEqual(cv::norm(kf->mK - pCameraParams_->mK), 0) && 
                    checkFloatEqual(cv::norm(kf->mDistCoef - pCameraParams_->mDistCoef), 0), 
                    "Camera parameters are not the same for all the keyframes \n" 
                    << "kf mK: " << kf->mK << "\n"
                    << "kf mDistCoef: " << kf->mDistCoef << "\n"
                    << "cam mK: " << pCameraParams_->mK << "\n"
                    << "cam mDistCoef: " << pCameraParams_->mDistCoef);
    }
#endif 

    // Fill matrix with matrix grid points
    int N = (int) ceil( float(depthSize.height * depthSize.width)/skDownsampleStep);
    matCamGridPoints_ = cv::Mat(N, 2, CV_32F);

    // Fill matrix with indices of points. -1 means invalid
    cv::Mat matPointIdxs = cv::Mat_<int>(depthSize.height, depthSize.width, -1); 
    
    // generate points on the image plane 
    int ii = 0;
    for (int m = 0; m < depthSize.height; m += skDownsampleStep)
    {
        for (int n = 0; n < depthSize.width; n += skDownsampleStep, ii++)
        {
            matCamGridPoints_.at<float>(ii, 0) = n; // x
            matCamGridPoints_.at<float>(ii, 1) = m; // y
            matPointIdxs.at<int>(m,n) = ii;   
        }
    }

#if 1
    // undistort grid points: these are used for backprojections while 'distorted'(original) coordinates are used to read color and registered depth 
    if (distCoef.at<float>(0) != 0.0)
    {
        cout << "undistorting grid points" << endl;
        // Undistort grid points
        matCamGridPoints_ = matCamGridPoints_.reshape(2);
        cv::undistortPoints(matCamGridPoints_, matCamGridPoints_, K, distCoef, cv::Mat(), K);
        matCamGridPoints_ = matCamGridPoints_.reshape(1);
    }
#endif
    
    // vector of neighbors indexes
    vCamGridPointsNeighborsIdxs_.resize(N);
    
    const Settings* settings = Settings::instance();
    Eigen::Matrix3f R_u1_r1; // from rectified to unrectified 
    if(PointCloudKeyFrame<PointT>::skbNeedRectification){    
        cv::cv2eigen(settings->R_r1_u1().t(), R_u1_r1);    
    }

    const bool bNeedRectification = PointCloudKeyFrame<PointT>::skbNeedRectification;

    // back project the points on the camera plane z=1
    ii = 0;
    for (int m = 0; m < depthSize.height; m += skDownsampleStep)
    {
        for (int n = 0; n < depthSize.width; n += skDownsampleStep, ii++)
        {
            Eigen::Vector3f p((matCamGridPoints_.at<float>(ii, 0) - cx) / fx, (matCamGridPoints_.at<float>(ii, 1) - cy) / fy, 1);

            if(bNeedRectification){
                p = R_u1_r1 * p;
                p /= p.z();
            }
            matCamGridPoints_.at<float>(ii, 0) = p.x();
            matCamGridPoints_.at<float>(ii, 1) = p.y();
            
            for(int q=0;q<NeighborhoodT::kSize;q++)
            {
                int elem_m = m + NeighborhoodT::dm[q]*skDownsampleStep;
                int elem_n = n + NeighborhoodT::dn[q]*skDownsampleStep; 
                if( (elem_m >=0) && (elem_m < depthSize.height) && (elem_n>=0) && (elem_n<depthSize.width) )
                {   
                    vCamGridPointsNeighborsIdxs_[ii].push_back(matPointIdxs.at<int>(elem_m,elem_n));
                }
                else
                {
                    vCamGridPointsNeighborsIdxs_[ii].push_back(-1);
                }
            }
        }
    }    
}




static cv::Vec3b labelToColor(const unsigned int label)
{
    static const uint ICOLOR = 5;
    static const uint PRIME_NUMBER=997u;
    
    //static const int DELTACOLOR = (256/ICOLOR);
    static const uint minColorVal = uint(256u/ICOLOR);
    static const uint rangeColor  = (256u/ICOLOR)*(ICOLOR-1u);
    
    if(label == 0) 
        return cv::Vec3b(0u,0u,0u);

    uint numColor = (label*PRIME_NUMBER);

    const uint r = minColorVal + uint(numColor%rangeColor);
    numColor = numColor/rangeColor;
    const uint g = minColorVal + uint(numColor%rangeColor);
    numColor = numColor/rangeColor;
    const uint b = minColorVal + uint(numColor%rangeColor);
    return cv::Vec3b(b,g,r); 
}


PointCloudMapping::PointCloudT::Ptr PointCloudMapping::GeneratePointCloudInCameraFrameBGRA(KeyFramePtr& kf, 
        cv::Mat& color, cv::Mat& depth, cv::Mat& pixelToPointIndex, std::vector<unsigned int>& segmentsCardinality )
{
    PointCloudT::Ptr cloud_camera(new PointCloudT());

    InitCamGridPoints(kf, depth.size());
    
    int kfid = kf->mnId; 
    
    TICKCLOUD("PC::DepthFilter");  
    if(pPointCloudMapParameters_->bFilterDepthImages) 
    {
        FilterDepthimage(depth,  pPointCloudMapParameters_->depthFilterDiameter, pPointCloudMapParameters_->depthFilterSigmaDepth, pPointCloudMapParameters_->depthSigmaSpace);
    }
    TOCKCLOUD("PC::DepthFilter");  
    
    static const int N = (int) ceil( float(depth.rows * depth.cols)/skDownsampleStep);
    cloud_camera->reserve(N); 
    
    pixelToPointIndex = cv::Mat_<int>(depth.rows, depth.cols, -1); // -1 means invalid
    
#if COMPUTE_NORMALS
    std::vector<int> idxCloud(N,-1); // -1 means invalid
#endif
         
    const cv::Vec2f* grid_point = matCamGridPoints_.ptr<cv::Vec2f>(0); 
    
    int ii = 0;
    for (int m = 0; m < depth.rows; m += skDownsampleStep)
    {
        const float* depth_row_m = depth.ptr<float>(m);
        const uchar* color_row_m = color.ptr<uchar>(m);
        int* pixelToPointIndex_row_m = pixelToPointIndex.ptr<int>(m);
        
        for (int n = 0; n < depth.cols; n += skDownsampleStep, ii++)
        {
            const float& d = depth_row_m[n];

            if ((d > pPointCloudMapParameters_->minDepthDistance) && (d < pPointCloudMapParameters_->maxDepthDistance))
            {
                PointT p;
                p.z = d;
                //p.x = mat_cam_grid_points_.at<float>(ii, 0) * d;
                //p.y = mat_cam_grid_points_.at<float>(ii, 1) * d;
                p.x = grid_point[ii][0] * d;
                p.y = grid_point[ii][1] * d;
                
                const int n3 = n * 3;
                // write colors into BGRA (invert for drawing in openGL)
                p.r = color_row_m[n3]; // B
                p.g = color_row_m[n3 + 1]; // G
                p.b = color_row_m[n3 + 2]; // R
                //p.a = 255; 
                
                PointUtils::setKFid(p,kfid);
                PointUtils::updateDepth(d,p);

                cloud_camera->points.push_back(p);

#if COMPUTE_NORMALS                
                const size_t index = cloud_camera->points.size()-1;
                idxCloud[ii] = index;
                pixelToPointIndex_row_m[n] = index; 
#endif          
            }

        }
    }
    
#if COMPUTE_NORMALS
    ii = 0;
    for (int m = 0; m < depth.rows; m += skDownsampleStep)
    {        
        for (int n = 0; n < depth.cols; n += skDownsampleStep, ii++)
        {
            if(idxCloud[ii]>=0) // central point is valid 
            {
                Eigen::Vector3d normal(0.d,0.d,0.d);  
                PointT& pc = cloud_camera->points[idxCloud[ii]];
                const Eigen::Vector3d vpc(pc.x,pc.y,pc.z);
                // get the other points in the neighborhood 
                for(int kk=NeighborhoodT::kiStartNormal;kk<NeighborhoodT::kSize;kk+=NeighborhoodT::kDiNormal)
                {
                    const int& point1IdxGrid = vCamGridPointsNeighborsIdxs_[ii][kk];
                    const int& point2IdxGrid = vCamGridPointsNeighborsIdxs_[ii][(kk+NeighborhoodT::kDiNormal)%NeighborhoodT::kSize];
                    if((point1IdxGrid<0) || (point2IdxGrid<0)) continue; 
                    
                    const int& point1IdxCloud = idxCloud[point1IdxGrid];
                    const int& point2IdxCloud = idxCloud[point2IdxGrid];
                    if((point1IdxCloud<0) || (point2IdxCloud<0)) continue;

                    const PointT& p1 = cloud_camera->points[point1IdxCloud];
                    const PointT& p2 = cloud_camera->points[point2IdxCloud];
                    const Eigen::Vector3d vp1(p1.x, p1.y, p1.z);
                    const Eigen::Vector3d vp2(p2.x, p2.y, p2.z);
                    //normal += (vp1-vpc).cross(vp2-vpc).normalized();  // uniform weighting
                    normal += (vp1-vpc).cross(vp2-vpc); // area weighting (we multiply for the norm which is 2*SpannedArea)
                }
                normal.normalize();
                pc.normal_x = normal[0];
                pc.normal_y = normal[1];
                pc.normal_z = normal[2];
            }
        }
    }
#endif    
    
    
#if COMPUTE_SEGMENTS

    if(pPointCloudMapParameters_->bSegmentationOn)
    {        
        static const int downsampleRows = (int) ceil( float(depth.rows)/skDownsampleStep);
        static const int downsampleCols = (int) ceil( float(depth.cols)/skDownsampleStep);
        
        const std::vector<cv::line_descriptor_c::KeyLine>& keyLines = kf->mvKeyLines;  // draw detected raw keylines 
        cv::Mat& linesImg = vecImages_[3].img;
        vecImages_[3].name = "lines";
        //linesImg = cv::Mat_<uchar>::zeros(depth.rows, depth.cols);
        linesImg = cv::Mat_<uchar>::zeros(downsampleRows, downsampleCols);
        vecImages_[3].bReady = true;  // comment this in order to hide the lines image
        for(size_t ii=0, iiEnd=keyLines.size(); ii<iiEnd; ii++)
        {
            cv::line(linesImg, keyLines[ii].getStartPoint()/skDownsampleStep, keyLines[ii].getEndPoint()/skDownsampleStep,cv::Scalar(255), pPointCloudMapParameters_->segmentationLineDrawThinckness);
        }
       
        vecImages_[0].name = "fi";
        vecImages_[1].name = "fi th";
        vecImages_[2].name = "delta ";
        cv::Mat& matFi = vecImages_[0].img;
        cv::Mat& matFiBinaryTh = vecImages_[1].img;
        cv::Mat& matGamma = vecImages_[2].img;
        matFi   = cv::Mat_<uchar>::zeros(downsampleRows, downsampleCols);
        matFiBinaryTh = cv::Mat_<uchar>::zeros(downsampleRows, downsampleCols);
        matGamma      = cv::Mat_<uchar>::zeros(downsampleRows, downsampleCols);
        vecImages_[0].bReady = true;
        vecImages_[1].bReady = true;
        vecImages_[2].bReady = true;

        ii = 0;
        for (int m = 0, md = 0; m < depth.rows; m += skDownsampleStep, md++)
        {        
            for (int n = 0, nd = 0; n < depth.cols; n += skDownsampleStep, ii++, nd++)
            {
                if(idxCloud[ii]>=0) // central point is valid 
                {
                    float minFi = std::numeric_limits<float>::max();  
                    float maxDelta = 0; 
                    PointT& pc = cloud_camera->points[idxCloud[ii]];
                    const Eigen::Vector3d vpc(pc.x,pc.y,pc.z);
                    const Eigen::Vector3d normal(pc.normal_x,pc.normal_y,pc.normal_z);
                    if(pc.z > pPointCloudMapParameters_->sementationMaxDepth) continue; 

                    // get the other points in the neighborhood 
                    for(int kk=0;kk<NeighborhoodT::kSize;kk++)
                    {
                        const int& point1IdxGrid = vCamGridPointsNeighborsIdxs_[ii][kk];
                        if(point1IdxGrid<0) continue; 

                        const int& point1IdxCloud = idxCloud[point1IdxGrid];
                        if(point1IdxCloud<0) continue;

                        const PointT& p1 = cloud_camera->points[point1IdxCloud];
                        const Eigen::Vector3d vp1(p1.x, p1.y, p1.z);
                        const Eigen::Vector3d deltav = (vp1-vpc);
                        float fi = 1;
                        const float weigth = Utils::SigmaZminOverSigmaZ(pc.z);
#if 1
                        const float deltavDotNormal = deltav.dot(normal);
                        if(deltavDotNormal >=0)
                        {
                            const Eigen::Vector3d normal1(p1.normal_x,p1.normal_y,p1.normal_z);
                            fi = normal1.dot(normal); 
                        }
#else
                        const Eigen::Vector3d normal1(p1.normal_x,p1.normal_y,p1.normal_z);
                        fi = normal1.dot(normal); 
                        if(deltav.dot(normal)<0)
                        {
                            fi *= fi;
                        }
#endif
                        if(minFi>fi) minFi = fi; 

                        const float delta = deltav.norm()*weigth;
                        //const float delta = fabs(deltavDotNormal)*weigth;
                        if(maxDelta < delta) maxDelta = std::min(delta,1.0f);
                    }

                    matFi.at<uchar>(md,nd) = static_cast<uchar>(std::min(minFi,1.0f)*255); 
                    if(maxDelta > pPointCloudMapParameters_->segmentationMaxDelta)
                        matGamma.at<uchar>(md,nd) = 255;
                    
                    const bool lineEdge = (linesImg.at<uchar>(md,nd) == 255);
                    
                    // here we mark the areas which are supposed to be convex and made of contiguous vertices
                    if( (minFi > pPointCloudMapParameters_->segmentationMinFi) && (maxDelta <= pPointCloudMapParameters_->segmentationMaxDelta) && !lineEdge) 
                    {
                        matFiBinaryTh.at<uchar>(md,nd) = 255;
                    }
                }
            }
        }
        

        TICKCLOUD("erosion-dilation");        
        if(pPointCloudMapParameters_->bSegmentationErosionDilationOn)
        {
            /// < apply erosion dilation 
            const int erosionSize = 1; 
            const int erosionElementSize = 2 * erosionSize + 1;
            const cv::Mat elementErosion = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erosionElementSize, erosionElementSize));
            const int dilationSize = 1;  
            const int dilationElementSize = 2 * dilationSize + 1;
            const cv::Mat elemenDilation = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(dilationElementSize, dilationElementSize));
            cv::Mat erosionOut; 
            cv::erode(matFiBinaryTh,erosionOut,elementErosion); 
            cv::dilate(erosionOut,matFiBinaryTh,elemenDilation);
            //cv::Mat openOut; 
            //cv::morphologyEx(matFiBinaryTh,openOut,cv::MORPH_OPEN,elementErosion);
            //cv::morphologyEx(openOut,matFiBinaryTh,cv::MORPH_CLOSE,elementErosion);
            
        }
        TOCKCLOUD("erosion-dilation");
                    
        /// < find connected components 
        cv::Mat labelImage(matFiBinaryTh.size(), CV_32S);
        cv::Mat stats;
        cv::Mat centroids;
        int nLabels = cv::connectedComponentsWithStats(matFiBinaryTh, labelImage, stats, centroids, 8);
                
        std::vector<cv::Vec3b> colors(nLabels);
        colors[0] = cv::Vec3b(0, 0, 0);//background
        
        std::vector<bool> isLabelValid(nLabels,false);
        
        for(int label = 1; label < nLabels; ++label) /// < N.B.: starting from 1!
        {
            const int area  = stats.at<int>(label, cv::CC_STAT_AREA);
            if(area > pPointCloudMapParameters_->segmentationSingleDepthMinComponentArea)
            {
                isLabelValid[label] = true;
                
                //colors[label] = cv::Vec3b( (rand()&255), (rand()&255), (rand()&255) );
                colors[label] = labelToColor(label);
            }
        }

        segmentsCardinality = std::vector<unsigned int>(nLabels,0); 
        
        cv::Mat& connectedComponents = vecImages_[4].img;
        vecImages_[4].name = "connected components";
        connectedComponents = cv::Mat_<cv::Vec3b>::zeros(downsampleRows, downsampleCols); //cv::Mat(matFiBinaryTh.size(), CV_8UC3);
        
        /// < label current point cloud  
        ii = 0;
        for (int m = 0, md = 0; m < depth.rows; m += skDownsampleStep, md++)
        {        
            for (int n = 0, nd = 0; n < depth.cols; n += skDownsampleStep, ii++, nd++)
            {
                const int& label = labelImage.at<int>(md, nd);
                
                if(!isLabelValid[label]) continue;
                
                cv::Vec3b &pixel = connectedComponents.at<cv::Vec3b>(md, nd);
                pixel = colors[label];
                                                
                if(idxCloud[ii]>=0) // point is valid 
                {
                    PointT& pc = cloud_camera->points[idxCloud[ii]];
                    pc.label = label;
                    
                    segmentsCardinality[label]++;
                }
            }
        }
        vecImages_[4].bReady = true;  
        
#define PRINT_CARDINALITIES 0
#if PRINT_CARDINALITIES
        // filter out components  with few elements     
        std::cout << "num components: " << nLabels << std::endl; 
        for(int label=0; label< nLabels; label++)
        {
            if(!isLabelValid[label]) continue;
            const int numElements = segmentsCardinality[label];
            std::cout <<" component["<<label<<"]: " <<  numElements << std::endl; 
        }
#endif
    
    }
            
#endif     

    cloud_camera->is_dense = true; // no points  are invalid!
    cloud_camera->header.stamp = TimeUtils::getTimestampfromSec(kf->mTimeStamp);

    return cloud_camera;

}

PointCloudMapping::PointCloudT::Ptr PointCloudMapping::GeneratePointCloudInCameraFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyframe)
{

    if (!pcKeyframe->bCloudReady)
    {
        PointCloudT::Ptr p_cloud_camera = GeneratePointCloudInCameraFrameBGRA(pcKeyframe->pKF, pcKeyframe->imgColor, pcKeyframe->imgDepth, pcKeyframe->imgPointIndex, pcKeyframe->labelMap.GetScanImgLabelsCardinality());
        pcKeyframe->SetCloud(p_cloud_camera);
        return p_cloud_camera;
    }
    else
    {
        return pcKeyframe->pCloudCamera;
    }
}

PointCloudMapping::PointCloudT::Ptr PointCloudMapping::GeneratePointCloudInWorldFrame(PointCloudKeyFrame<PointT>::Ptr pcKeyframe)
{
    GeneratePointCloudInCameraFrame(pcKeyframe);

    Eigen::Isometry3d T = PLVS2::Converter::toSE3Quat(pcKeyframe->pKF->GetPose());

    PointCloudT::Ptr p_cloud_world(new PointCloudT);
    //pcl::transformPointCloud(*(pcKeyframe->pCloudCamera), *p_cloud_world, T.inverse().matrix());
#if 0
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3d, double>(*(pcKeyframe->pCloudCamera), *p_cloud_world, T.inverse());
#else 
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3f, float>(*(pcKeyframe->pCloudCamera), *p_cloud_world, T.inverse().cast<float>());
#endif     
    p_cloud_world->is_dense = false;

    cout << "generate point cloud for kf " << pcKeyframe->pKF->mnId << ", size=" << p_cloud_world->points.size() << endl;
    return p_cloud_world;
}

PointCloudMapping::PointCloudT::Ptr PointCloudMapping::GetMap()
{
    std::chrono::milliseconds timeout(kTimeoutForWaitingMapMs);
    
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_, timeout);
    if (lck.owns_lock())
    {
        pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
        return pPointCloudMap_->GetMapWithTimeout(timeout);
    }
    else
    {
        return 0;
    }
}

void PointCloudMapping::GetMap(typename PointCloudT::Ptr& pCloud, typename PointCloudT::Ptr& pCloudUnstable, std::vector<unsigned int>& faces, bool copyUnstable)
{
    //std::cout << "PointCloudMapping::GetMap()" << std::endl;
    const std::chrono::milliseconds timeout(kTimeoutForWaitingMapMs);
    
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_, timeout);
    if (lck.owns_lock())
    {
        pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
        //std::cout << "PointCloudMapping::GetMap() - getting map in " << timeout.count() << " milliseconds " << std::endl;
        pPointCloudMap_->GetMapWithTimeout(pCloud, pCloudUnstable, faces, timeout, copyUnstable);
    }
    else
    {
        pCloud = 0;
        pCloudUnstable = 0;
        faces.clear();
    }
}

std::uint64_t PointCloudMapping::GetMapTimestamp()
{
    unique_lock<mutex> lck(pointCloudTimestampMutex_);
    return pointCloudTimestamp_;
}

void PointCloudMapping::SaveMap()
{
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);
    pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
    
    std::stringstream map_name;
    map_name << "volumetric_map_out_" << mnSaveMapCount_++ << ".ply";

    return pPointCloudMap_->SaveMap(map_name.str());
}

void PointCloudMapping::LoadMap()
{
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);
    
}

void PointCloudMapping::FilterDepthimage(cv::Mat &image,  const int diamater , const double sigmaDepth , const double sigmaSpace )
{
    std::cout<<"filtering depth" <<  std::endl;  
    
#if 0
    static const float kDeltaDepth = sigmaDepth; 
    static const int kDeltaPixel = diamater; 
    
    cv::Mat input = image.clone();
    for (int x = 0; x < image.cols; x++)
    {
        for (int y = 0; y < image.rows; y++)
        {
            float& val = input.at<float>(y, x);
            if (std::isfinite(val))
            {
                float sum   = 0.0f;
                int count = 0;
                for (int dx = -kDeltaPixel; dx <= kDeltaPixel; dx++)
                {
                    for (int dy = -kDeltaPixel; dy <= kDeltaPixel; dy++)
                    {
                        const int otherx =  x + dx;
                        const int othery =  y + dy;
                        if ( (otherx >=0 ) && (otherx<image.cols) && (othery>=0) && (othery<image.rows) )
                        {
                            const float otherval = input.at<float>(othery, otherx);
                            if (std::isfinite(otherval) && fabs(val - otherval) < kDeltaDepth)
                            {
                                sum   += otherval;
                                count ++;
                            }
                        }
                    }
                }
                val = sum / std::max(count,(int)1);
            }
        }
    }
  
#else
    

    cv::Mat input = image.clone();
    
    // eliminate nans (otherwise bilateral filter returns a segmentation fault)
    for (int m = 0; m < input.rows; m++)
    {
        float* input_row_m = input.ptr<float>(m);        
        for (int n = 0; n < input.cols; n++)
        {
            float& d = input_row_m[n];
            if (std::isnan(d))
                d = kMaxDepthDistance; //std::numeric_limits<float>::max();
            
        }
    }
    cv::bilateralFilter(input, image, diamater, sigmaDepth, sigmaSpace);
    //cv::medianBlur(input, image, diamater);

#endif
}


void PointCloudMapping::SetCameraCalibration(float fx, float fy, float cx, float cy )
{
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);    
    pPointCloudMap_ = mpPointCloudAtlas->GetCurrentMap();
    
    pCameraParams_->fx = fx;
    pCameraParams_->fy = fy;
    pCameraParams_->cx = cx;
    pCameraParams_->cy = cy;
    
    /// < NOTE: we assume frames are already rectified and depth camera and rgb camera have the same projection models 
    pPointCloudMap_->SetColorCameraModel(*pCameraParams_);
    pPointCloudMap_->SetDepthCameraModel(*pCameraParams_);    
}


} //namespace PLVS2





//PointCloudMapping::PointCloudT::Ptr PointCloudMapping::GeneratePointCloud(KeyFramePtr kf, cv::Mat& color, cv::Mat& depth)
//{
//    PointCloudT::Ptr cloud_camera(new PointCloudT());
//    
//    // point cloud is null ptr
//    for (int m = 0; m < depth.rows; m += downsampleStep_)
//    {
//        for (int n = 0; n < depth.cols; n += downsampleStep_)
//        {
//            float d = depth.ptr<float>(m)[n];
//            if ((d < pPointCloudMapParameters_->minDepthDistance) || (d > pPointCloudMapParameters_->maxDepthDistance))
//                continue;
//
//            PointT p;
//            p.z = d;
//            p.x = (n - kf->cx) * p.z / kf->fx;
//            p.y = (m - kf->cy) * p.z / kf->fy;
//
//            int n3 = n * 3;
//            p.b = color.ptr<uchar>(m)[n3];
//            p.g = color.ptr<uchar>(m)[n3 + 1];
//            p.r = color.ptr<uchar>(m)[n3 + 2];
//            //p.a = 255;            
//
//            cloud_camera->points.push_back(p);
//        }
//    }
//    
//    cloud_camera->header.stamp = getTimestampfromSec(kf->mTimeStamp);
//
//    Eigen::Isometry3d T = PLVS2::Converter::toSE3Quat(kf->GetPose());
//    PointCloudT::Ptr cloud(new PointCloudT);
//    pcl::transformPointCloud(*cloud_camera, *cloud, T.inverse().matrix());
//    cloud->is_dense = false;
//
//    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
//    return cloud;
//}
