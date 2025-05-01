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

#include "KeyFrame.h"
#include "Converter.h"

#include "PointCloudMapping.h"
#include "Map.h"
#include "LocalMapping.h"
#include "ColorOctomapServer.h"
#include "PointCloudMap.h"
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

namespace PLVS
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



PointCloudMapping::PointCloudMapping(const string &strSettingPath, Map* map, LocalMapping* localMap): 
mpMap(map), mpLocalMapping(localMap), bInitCamGridPoints_(false),bFinished_(true),mnSaveMapCount(0)
{
    cv::FileStorage fsSettings(strSettingPath, cv::FileStorage::READ);

    // < StereoDense
    std::cout << std::endl  << "StereoDense Parameters: " << std::endl; 
    std::string stereoDenseStringType = Utils::GetParam(fsSettings, "StereoDense.type", std::string("libelas")); 
    if (stereoDenseStringType == "libelas") PointCloudKeyFrame<PointT>::ksStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibelas;
    if (stereoDenseStringType == "libsgm") PointCloudKeyFrame<PointT>::ksStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibsgm;  
    if (stereoDenseStringType == "opencv") PointCloudKeyFrame<PointT>::ksStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCV; 
    if (stereoDenseStringType == "opencvcuda") PointCloudKeyFrame<PointT>::ksStereoLibrary = PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCVCuda;  
    
    // < PointCloudMapping
    std::cout << std::endl  << "PointCloudMapping Parameters: " << std::endl;   
    bActive_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.on", 0)) != 0;
    
    numKeyframesToQueueBeforeProcessing_ = Utils::GetParam(fsSettings, "PointCloudMapping.numKeyframesToQueueBeforeProcessing", kNumKeyframesToQueueBeforeProcessing);
    
    skDownsampleStep = Utils::GetParam(fsSettings, "PointCloudMapping.downSampleStep", 2);
    
    resolution_ = Utils::GetParam(fsSettings, "PointCloudMapping.resolution", kGridMapDefaultResolution);
    
    std::cout << "PointCloudMapping::PointCloudMapping() - resolution: " <<  resolution_ << std::endl;

    maxDepthDistance_ = Utils::GetParam(fsSettings, "PointCloudMapping.maxDepth", kMaxDepthDistance);
    minDepthDistance_ = Utils::GetParam(fsSettings, "PointCloudMapping.minDepth", kMinDepthDistance);
    
    KeyFrame::skFovCenterDistance = 0.5*(minDepthDistance_ + maxDepthDistance_);
    
    imageDepthScale_ = Utils::GetParam(fsSettings, "DepthMapFactor", 1.);

    // read parameters for specific map models 
    bUseCarving_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.useCarving", 1)) != 0;
    
    bool bRemoveUnstablePoints = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.removeUnstablePoints", 1)) != 0;
    
    bool bResetOnSparseMapChange = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.resetOnSparseMapChange", 1)) != 0;
    bCloudDeformationOnSparseMapChange_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.cloudDeformationOnSparseMapChange", 0)) != 0;
    if(bCloudDeformationOnSparseMapChange_) bResetOnSparseMapChange = false;
    
    int point_counter_threshold = Utils::GetParam(fsSettings, "PointCloudMapping.pointCounterThreshold", kGridMapDefaultPointCounterThreshold);

    PointCloudMapVoxblox<PointT>::skIntegrationMethod = Utils::GetParam(fsSettings, "PointCloudMapping.voxbloxIntegrationMethod", PointCloudMapVoxblox<PointT>::skIntegrationMethod);
    
    // read the desired map model 
    std::string pointCloudMapTypeStringDefault = "octree_point";
    pointCloudMapType_ = kVoxelGrid;

    std::string pointCloudMapStringType = Utils::GetParam(fsSettings, "PointCloudMapping.type", pointCloudMapTypeStringDefault);
    std::cout << "point cloud map type: " << pointCloudMapStringType << std::endl;

    if (pointCloudMapStringType == "voxelgrid")
    {
        //pPointCloudMap_ = std::make_shared<PointCloudMapVoxelGridFilter<PointT> >(resolution_);
        pPointCloudMap_ = std::make_shared<PointCloudMapVoxelGridFilterActive<PointT> >(resolution_);
        pointCloudMapType_ = kVoxelGrid;
    }
    else if (pointCloudMapStringType == "octomap")
    {
        pPointCloudMap_ = std::make_shared<PointCloudMapOctomap<PointT> >(resolution_, maxDepthDistance_);
        pointCloudMapType_ = kOctomap;
    }
    else if (pointCloudMapStringType == "octree_point")
    {
        pPointCloudMap_ = std::make_shared<PointCloudMapOctreePointCloud<PointT> >(resolution_, point_counter_threshold, bUseCarving_);
        pointCloudMapType_ = kOctreePoint;
    }
    else if (pointCloudMapStringType == "chisel")
    {
        pPointCloudMap_ = std::make_shared<PointCloudMapChisel<PointT> >(resolution_, minDepthDistance_, maxDepthDistance_, bUseCarving_);
        pointCloudMapType_ = kChisel;
    }
    else if (pointCloudMapStringType == "fastfusion")
    {
        pPointCloudMap_ = std::make_shared<PointCloudMapFastFusion<PointT> >(resolution_, minDepthDistance_, maxDepthDistance_, bUseCarving_);
        pointCloudMapType_ = kFastFusion;
    }
    else if (pointCloudMapStringType == "voxblox")
    {
        pPointCloudMap_ = std::make_shared<PointCloudMapVoxblox<PointT> >(resolution_, minDepthDistance_, maxDepthDistance_, bUseCarving_);
        pointCloudMapType_ = kVoxblox;
    }    
    else
    {
        // default 
        pPointCloudMap_ = std::make_shared<PointCloudMapVoxelGridFilter<PointT> >(resolution_);
        pointCloudMapType_ = kVoxelGrid;
        
        std::cout << "PointCloudMapping::PointCloudMapping() - selecting default point map PointCloudMapVoxelGridFilter " << std::endl; 
    }


    /// < NOTE: here we manage a simple model (without distortion) which is used for projecting point clouds;
    /// <       it assumes input rectified images; in particular chisel framework works under these assumptions
    pCameraParams = std::make_shared<CameraModelParams>();
    pCameraParams->fx = fsSettings["Camera.fx"];
    pCameraParams->fy = fsSettings["Camera.fy"];
    pCameraParams->cx = fsSettings["Camera.cx"];
    pCameraParams->cy = fsSettings["Camera.cy"];
    pCameraParams->width = fsSettings["Camera.width"];
    pCameraParams->height = fsSettings["Camera.height"];
    pCameraParams->minDist = minDepthDistance_;
    pCameraParams->maxDist = maxDepthDistance_;
    
    /// < NOTE: we assume frames are already rectified and depth camera and rgb camera have the same projection models 
    pPointCloudMap_->SetColorCameraModel(*pCameraParams);
    pPointCloudMap_->SetDepthCameraModel(*pCameraParams);
    
    pointCloudTimestamp_ = 0;

    bKeyframesAvailable_ = false;
    bKeyframesToInsertInMap_ = false;
    
    vecImages_.resize(5);

    bFilterDepthImages_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.on", 0)) != 0;
    depthFilterDiamater_ = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.diameter", kDepthFilterDiamater);  
    depthFilterSigmaDepth_ = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.sigmaDepth", kDepthFilterSigmaDepth); 
    depthSigmaSpace_ = Utils::GetParam(fsSettings, "PointCloudMapping.filterDepth.sigmaSpace", kDepthSigmaSpace); 
    
    mbLoadDensemap_ = static_cast<int> (Utils::GetParam(fsSettings, "PointCloudMapping.loadMap", 0)) != 0;
    msLoadFilename = Utils::GetParam(fsSettings, "PointCloudMapping.loadFilename", std::string()); 
    
    cout << endl  << "Segmentation Parameters: " << endl; 
    bSegmentationOn_ = static_cast<int> (Utils::GetParam(fsSettings, "Segmentation.on", 0)) != 0;
    
    // < disable segmentation if you are not using octreepoint
    if( (bSegmentationOn_) && (pointCloudMapType_ != kOctreePoint) )
    {
        cout << endl  << "!!!WARNING: segmentation disabled without octree_point!!!" << endl; 
        bSegmentationOn_ = false; 
        
    }
    
    if(bSegmentationOn_)
    {
        std::cout << "WARNING: forcing numKeyframesToQueueBeforeProcessing to 1" << std::endl; 
        numKeyframesToQueueBeforeProcessing_ = 1; 
        
        std::cout << "WARNING: forcing filter depth on" << std::endl; 
        bFilterDepthImages_ = true; 
    }
    
    bSegmentationErosionDilationOn_ = static_cast<int> (Utils::GetParam(fsSettings, "Segmentation.erosionDilationOn", 1)) != 0;
    
    sementationMaxDepth_ = Utils::GetParam(fsSettings, "Segmentation.maxDepth", kSementationMaxDepth);
    segmentationMinFi_ = Utils::GetParam(fsSettings, "Segmentation.minFi", kSegmentationMinFi);
    segmentationMaxDelta_ = Utils::GetParam(fsSettings, "Segmentation.maxDelta", kSegmentationMaxDelta);
    segmentationSingleDepthMinComponentArea_ = Utils::GetParam(fsSettings, "Segmentation.singleDepth.minComponentArea", kSegmentationSingleDepthMinComponentArea);
    segmentationLineDrawThinckness_ = Utils::GetParam(fsSettings, "Segmentation.lineDrawThinckness", 2);
    
    segmentationLabelConfidenceThreshold_ = Utils::GetParam(fsSettings, "Segmentation.labelConfidenceThreshold", kSegmenentationLabelConfidenceThreshold);
   
    float maxAngleForNormalAssociation = Utils::GetParam(fsSettings, "Segmentation.maxAngleForNormalAssociation", 20);
    LabelMap::skLabelsMatchingMinOverlapPerc = Utils::GetParam(fsSettings, "Segmentation.labelsMatchingMinOverlapPerc", LabelMap::kLabelsMatchingMinOverlaPercDefault);
    LabelMap::skLabelsMatchingMinOverlapPoints = Utils::GetParam(fsSettings, "Segmentation.labelsMatchingMinOverlapPoints", LabelMap::kLabelsMatchingMinOverlapPointsDefault);
    GlobalLabelMap::skLabelsMatchingMinOverlapPerc = Utils::GetParam(fsSettings, "Segmentation.globalLabelsMatchingMinOverlapPerc", GlobalLabelMap::kLabelsMatchingMinOverlaPercDefault);
    
    pPointCloudMap_->SetDownsampleStep(skDownsampleStep);
    pPointCloudMap_->SetPerformCarving(bUseCarving_);
    pPointCloudMap_->SetRemoveUnstablePoints(bRemoveUnstablePoints);
    pPointCloudMap_->SetResetOnMapChange(bResetOnSparseMapChange);
    pPointCloudMap_->SetKfAdjustmentOnMapChange(bCloudDeformationOnSparseMapChange_);
    pPointCloudMap_->SetPerformSegmentation(bSegmentationOn_);
    
    pPointCloudMap_->SetMinCosForNormalAssociation(cos(maxAngleForNormalAssociation*M_PI/180.));
        
    // load dense map 
    if( !msLoadFilename.empty() &&  mbLoadDensemap_ )
    {
        pPointCloudMap_->LoadMap(msLoadFilename);
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

    cout << "receive a keyframe, id = " << pcKeyFrame->pKF->mnId << endl;
    if(baseKeyframeId_< 0) baseKeyframeId_ = pcKeyFrame->pKF->mnId; 

    unique_lock<mutex> lck(keyframesMutex_);

#if INIT_PCKF_ON_INSERT    
    pcKeyFrame->Init(); // N.B.: no memory sharing for color and depth  
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

    for (auto it=pcKeyframesIn_.begin(); it != pcKeyframesIn_.end(); )
    {
        PointCloudKeyFrame<PointT>::Ptr pcKeyframe = *it;
        cout << "Preparing keyframe, id = " << pcKeyframe->pKF->mnId << " (Bad: " << (int)pcKeyframe->pKF->isBad() << ", LBA count: " << pcKeyframe->pKF->mnLBACount <<  ") (#queued: " << pcKeyframesIn_.size() << ")" << endl;
        
#if !INIT_PCKF_ON_INSERT          
        pcKeyframe->Init(); // N.B.: no memory sharing for color and depth  
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

    set<KeyFramePtr> set_pKFs = mpMap->GetSetKeyFrames();
    mpLocalMapping->AddNewKeyFramesToSet(set_pKFs);
    

    unique_lock<recursive_timed_mutex> lck_globalMap(pointCloudMutex_);
    unique_lock<mutex> lck_pcKeyframes(pcKeyframesMutex_);

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
    pcl::uint64_t timestamp = pPointCloudMap_->GetMapTimestamp();

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
            cout << "integrating point cloud for kf " << pcKeyframe->pKF->mnId << "/" << (pcKeyframes_.size()-1) + baseKeyframeId_ << endl;
   
            PointCloudT::Ptr pCloudCamera = GeneratePointCloudInCameraFrame(pcKeyframe);  
            
            std::shared_ptr<PointCloudMapInput<PointT> > pPointCloudMapInput = std::make_shared<PointCloudMapInput<PointT> >(
                    pcKeyframe, minDepthDistance_, maxDepthDistance_, imageDepthScale_, pCloudCamera->header.stamp);
                    
            switch (pointCloudMapType_) 
            {
            case kChisel:            
                // two possible modes   
                if(bUseCarving_)
                    pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloudAndDepthImage;
                else
                    pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloud;                    
                //pPointCloudMapInput->type = PointCloudMapInput<PointT>::kColorAndDepthImages;  /// < NOTE: much sloweeer that inserting with pointcloud!!!!
                break;
                
            case kFastFusion:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kColorAndDepthImages;
                break;
                   
            case kOctreePoint:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloudAndDepthImage;
                break;

            default:
                pPointCloudMapInput->type = PointCloudMapInput<PointT>::kPointCloud;
            }
            
            TICKCLOUD("PC::InsertData");
                    
            pPointCloudMap_->InsertData(pPointCloudMapInput);
            
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
    std::cout << "PointCloudMapping::reset() - start " << std::endl;

    unique_lock<mutex> lck(keyframesMutex_); // block the insertion of new frames

    bKeyframesAvailable_ = false;
    bKeyframesToInsertInMap_ = false; 
    
    pcKeyframesIn_.clear();

    {
        unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);
        pPointCloudMap_->Clear();
    }

    {
        unique_lock<mutex> lck(pcKeyframesMutex_);
        pcKeyframes_.clear();
        lastKeyframeIndex_ = 0;
        
        pcKeyframesToReinsert_.clear();
    }
    std::cout << "PointCloudMapping::reset() - end" << std::endl;    
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
        //pPointCloudMap_->Clear();
        pPointCloudMap_->OnMapChange();
    }

    if(!bCloudDeformationOnSparseMapChange_)
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
void PointCloudMapping::InitCamGridPoints(KeyFramePtr& kf, cv::Mat& depth)
{
    if (bInitCamGridPoints_) return;
    bInitCamGridPoints_ = true;

    //std::cout << "PointCloudMapping::initCamGridPoints() ***************" << std::endl;

    // Fill matrix with matrix grid points
    int N = (int) ceil( float(depth.rows * depth.cols)/skDownsampleStep);
    matCamGridPoints_ = cv::Mat(N, 2, CV_32F);

    // Fill matrix with indices of points. -1 means invalid
    cv::Mat matPointIdxs = cv::Mat_<int>(depth.rows, depth.cols, -1); 
    
    // generate points on the image plane 
    int ii = 0;
    for (int m = 0; m < depth.rows; m += skDownsampleStep)
    {
        for (int n = 0; n < depth.cols; n += skDownsampleStep, ii++)
        {
            matCamGridPoints_.at<float>(ii, 0) = n; // x
            matCamGridPoints_.at<float>(ii, 1) = m; // y
            
            matPointIdxs.at<int>(m,n) = ii;   
        }
    }

#if 1
    // undistort grid points: these are used for backprojections while 'distorted'(original) coordinates are used to read color and registered depth 
    if (kf->mDistCoef.at<float>(0) != 0.0)
    {
        cout << "undistorting grid points" << endl;
        // Undistort grid points
        matCamGridPoints_ = matCamGridPoints_.reshape(2);
        cv::undistortPoints(matCamGridPoints_, matCamGridPoints_, kf->mK, kf->mDistCoef, cv::Mat(), kf->mK);
        matCamGridPoints_ = matCamGridPoints_.reshape(1);
    }
#endif
    
    // vector of neighbors indexes
    vCamGridPointsNeighborsIdxs_.resize(N);
    
    // define neighborhood deltas 
    //int dm[4]={-1, 0, 1, 0};
    //int dn[4]={ 0,-1, 0, 1};
    
    // back project the points on the camera plane z=1
    ii = 0;
    for (int m = 0; m < depth.rows; m += skDownsampleStep)
    {
        for (int n = 0; n < depth.cols; n += skDownsampleStep, ii++)
        {
            matCamGridPoints_.at<float>(ii, 0) = (matCamGridPoints_.at<float>(ii, 0) - kf->cx) / kf->fx;
            matCamGridPoints_.at<float>(ii, 1) = (matCamGridPoints_.at<float>(ii, 1) - kf->cy) / kf->fy;
            
            for(int q=0;q<NeighborhoodT::kSize;q++)
            {
                int elem_m = m + NeighborhoodT::dm[q]*skDownsampleStep;
                int elem_n = n + NeighborhoodT::dn[q]*skDownsampleStep; 
                if( (elem_m >=0) && (elem_m < depth.rows) && (elem_n>=0) && (elem_n<depth.cols) )
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

    InitCamGridPoints(kf, depth);
    
    int kfid = kf->mnId; 
    
    TICKCLOUD("PC::DepthFilter");  
    if(bFilterDepthImages_) 
    {
        FilterDepthimage(depth,  depthFilterDiamater_, depthFilterSigmaDepth_, depthSigmaSpace_);
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

            if ((d > minDepthDistance_) && (d < maxDepthDistance_))
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

    if(bSegmentationOn_)
    {        
        static const int downsampleRows = (int) ceil( float(depth.rows)/skDownsampleStep);
        static const int downsampleCols = (int) ceil( float(depth.cols)/skDownsampleStep);
        
        const std::vector<cv::line_descriptor_c::KeyLine>& keyLines = kf->mvKeyLines; 
        cv::Mat& linesImg = vecImages_[3].img;
        vecImages_[3].name = "lines";
        //linesImg = cv::Mat_<uchar>::zeros(depth.rows, depth.cols);
        linesImg = cv::Mat_<uchar>::zeros(downsampleRows, downsampleCols);
        vecImages_[3].bReady = true;  // comment this in order to hide the lines image
        for(size_t ii=0, iiEnd=keyLines.size(); ii<iiEnd; ii++)
        {
            cv::line(linesImg, keyLines[ii].getStartPoint()/skDownsampleStep, keyLines[ii].getEndPoint()/skDownsampleStep,cv::Scalar(255),segmentationLineDrawThinckness_);
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
                    if(pc.z > sementationMaxDepth_) continue; 

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
                    if(maxDelta > segmentationMaxDelta_)
                        matGamma.at<uchar>(md,nd) = 255;
                    
                    const bool lineEdge = (linesImg.at<uchar>(md,nd) == 255);
                    
                    // here we mark the areas which are supposed to be convex and made of contiguous vertices
                    if( (minFi > segmentationMinFi_) && (maxDelta <= segmentationMaxDelta_) && !lineEdge) 
                    {
                        matFiBinaryTh.at<uchar>(md,nd) = 255;
                    }
                }
            }
        }
        

        TICKCLOUD("erosion-dilation");        
        if(bSegmentationErosionDilationOn_)
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
            if(area > segmentationSingleDepthMinComponentArea_)
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

    Eigen::Isometry3d T = PLVS::Converter::toSE3Quat(pcKeyframe->pKF->GetPose());

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

pcl::uint64_t PointCloudMapping::GetMapTimestamp()
{
    unique_lock<mutex> lck(pointCloudTimestampMutex_);
    return pointCloudTimestamp_;
}

void PointCloudMapping::SaveMap()
{
    unique_lock<recursive_timed_mutex> lck(pointCloudMutex_);

    std::stringstream map_name;
    map_name << "volumetric_map_out_" << mnSaveMapCount++ << ".ply";

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
    pCameraParams->fx = fx;
    pCameraParams->fy = fy;
    pCameraParams->cx = cx;
    pCameraParams->cy = cy;
    
    /// < NOTE: we assume frames are already rectified and depth camera and rgb camera have the same projection models 
    pPointCloudMap_->SetColorCameraModel(*pCameraParams);
    pPointCloudMap_->SetDepthCameraModel(*pCameraParams);    
}


} //namespace PLVS





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
//            if ((d < minDepthDistance_) || (d > maxDepthDistance_))
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
//    Eigen::Isometry3d T = PLVS::Converter::toSE3Quat(kf->GetPose());
//    PointCloudT::Ptr cloud(new PointCloudT);
//    pcl::transformPointCloud(*cloud_camera, *cloud, T.inverse().matrix());
//    cloud->is_dense = false;
//
//    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
//    return cloud;
//}
