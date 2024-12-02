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

//#define _DEBUG // for activating debugging in opencv

#include "PointCloudMapVoxelGridFilter.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "KeyFrame.h"
#include "KeyFrameSearchTree.h"

#define LET_ME_SEE_JUST_THE_LAST_CLOUD 0


namespace PLVS2
{

template<typename PointT>
const int PointCloudMapVoxelGridFilter<PointT>::kDefaultPointCounterThreshold = 0;

template<typename PointT>
PointCloudMapVoxelGridFilter<PointT>::PointCloudMapVoxelGridFilter(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params)
{
    voxel_.setLeafSize(params->resolution, params->resolution, params->resolution);
    //voxel_.setMinimumPointsNumberPerVoxel(point_counter_threshold);
}

template<typename PointT>
void PointCloudMapVoxelGridFilter<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    *(this->pPointCloud_) += *cloud_world;
}

template<typename PointT>
void PointCloudMapVoxelGridFilter<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    assert(pData->type == PointCloudMapInput<PointT>::kPointCloud);

    typename PointCloudT::Ptr pCloudWorld(new PointCloudT);
    Sophus::SE3f Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc; 
    
    this->TransformCameraCloudInWorldFrame(pData->pPointCloudKeyFrame->pCloudCamera, Converter::toIsometry3d(Twc), pCloudWorld);

#if !LET_ME_SEE_JUST_THE_LAST_CLOUD
    *(this->pPointCloud_) += *pCloudWorld;
#else
    this->pPointCloud_ = pCloudWorld; // just for testing in order to check the state of the last world pointcloud
    //*(this->pPointCloud_) = *(pData->pPointCloudKeyFrame->pCloudCamera); // just for testing in order to check the state of the last camera pointcloud
#endif
}

template<typename PointT>
int PointCloudMapVoxelGridFilter<PointT>::UpdateMap()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
#if !LET_ME_SEE_JUST_THE_LAST_CLOUD    
    typename PointCloudMap<PointT>::PointCloudT::Ptr new_cloud(new PointCloudT());
    this->voxel_.setInputCloud(this->pPointCloud_);
    this->voxel_.filter(*new_cloud);
    this->pPointCloud_->swap(*new_cloud);
#endif

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapVoxelGridFilter<PointT>::Clear()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
}

template<typename PointT>
void PointCloudMapVoxelGridFilter<PointT>::OnMapChange()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxelGridFilter<PointT>::OnMapChange() - point cloud map reset *** " << std::endl;
        PointCloudMap<PointT>::Clear();
    }
}


// =============================================================================

template<typename PointT>
PointCloudMapModel<PointT>::PointCloudMapModel()
{
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10
    pPointCloud_ = boost::make_shared< PointCloudT >();   
#else
    pPointCloud_ = std::make_shared< PointCloudT >();
#endif  
}

template<typename PointT>
void PointCloudMapModel<PointT>::InsertCloud(uint32_t kfid, typename PointCloudT::Ptr pCloud)
{
    const size_t modelSize = this->pPointCloud_->size();     
    
    const size_t cloudSize = pCloud->size();    
    PointListPtr pList(new PointList(cloudSize));  
    for(size_t ii=0;ii<cloudSize;ii++) (*pList)[ii] = ii+modelSize;
    mapKfidToPointList_[kfid] = pList; 
    
    *(this->pPointCloud_) += *pCloud;    
}

template<typename PointT>
void PointCloudMapModel<PointT>::UpdateCloud(typename PointCloudT::Ptr pCloud)
{
    mapKfidToPointList_.clear();
    this->pPointCloud_ = pCloud;   
    
    const size_t cloudSize = pCloud->size();
    for(size_t ii=0;ii<cloudSize;ii++)
    {
        const uint32_t& kfid=pCloud->points[ii].kfid; 
        /*if(mapKfidToPointList_.count(kfid)>0)
        {
            mapKfidToPointList_[kfid]->push_back(ii);
        }
        else
        {
            mapKfidToPointList_[kfid] = new PointList(1,ii);
        }*/
        auto it = mapKfidToPointList_.find(kfid);
        if(it!=mapKfidToPointList_.end())
        {
            it->second->push_back(ii);
        }
        else
        {
            mapKfidToPointList_[kfid] = new PointList(1,ii);
        }
                
    }
}

// =============================================================================


template<typename PointT>
const int PointCloudMapVoxelGridFilterActive<PointT>::kDefaultPointCounterThreshold = 0;
template<typename PointT>
const float PointCloudMapVoxelGridFilterActive<PointT>::kKdtreeSearchRangeFactor = 2.5;

template<typename PointT>
PointCloudMapVoxelGridFilterActive<PointT>::PointCloudMapVoxelGridFilterActive(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params)
{
    voxel_.setLeafSize(params->resolution, params->resolution, params->resolution);
    //voxel_.setMinimumPointsNumberPerVoxel(point_counter_threshold);
    
    pKfSearchTree_.reset(new KeyFrameSearchTree<>(params->resolution));     
}

template<typename PointT>
void PointCloudMapVoxelGridFilterActive<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    *(this->pPointCloud_) += *cloud_world;
}

template<typename PointT>
void PointCloudMapVoxelGridFilterActive<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    assert(pData->type == PointCloudMapInput<PointT>::kPointCloud);
        
    pKfSearchTree_->SetSearchRange(kKdtreeSearchRangeFactor*pData->maxRange);
    
    KeyFramePtr pKF = pData->pPointCloudKeyFrame->pKF;
    uint32_t kfid = pKF->mnId;    
    this->mapKfidPointCloudKeyFrame_[kfid] = pData->pPointCloudKeyFrame;    
    
    pKfSearchTree_->AddKeyFrame(pKF);
    pKfSearchTree_->GetCloseKeyFrames(pKF, setActiveKFs_, setActiveIds_); 

    typename PointCloudT::Ptr pCloudWorld(new PointCloudT);
    pData->pPointCloudKeyFrame->TwcIntegration = pData->pPointCloudKeyFrame->GetCameraPose(); 
    
    this->TransformCameraCloudInWorldFrame(pData->pPointCloudKeyFrame->pCloudCamera, Converter::toIsometry3d(pData->pPointCloudKeyFrame->TwcIntegration), pCloudWorld);
    
    *(this->pPointCloud_) += *pCloudWorld; // push back all points 
}

template<typename PointT>
int PointCloudMapVoxelGridFilterActive<PointT>::UpdateMap()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    
    // update Active Model (AM) and Inactive Model (IM): 
    //  - identify active KFs [DONE]
    //  - remove inactive points from AM and push them inside IM
    //      * iterate over all points of AM:
    //          - group them in pairs (KFID, list of points with kfid==KFID)  (?couldn't we maintain this when we integrate the new sensed cloud in AM?)
    //          - remove all pairs whose kfid is not present in setActiveIds_ and create a list of removed points 
    //      * push the removed points inside IM 
    //  - identify points of IM that have become active again => "active-again" points
    //  - recover/re-integrate in AM the identified "active-again" points
    //  - integrate new sensed cloud in AM    
 
    typename PointCloudMap<PointT>::PointCloudT::Ptr pNewCloud(new PointCloudT());
    this->voxel_.setInputCloud(this->pPointCloud_);
    this->voxel_.filter(*pNewCloud);
    this->pPointCloud_->swap(*pNewCloud);
    
    /*for(size_t ii=0,iiEnd=this->pPointCloud_->size(); ii<iiEnd; ii++)
    {
        std::cout << "point["<<ii<<"]: " << this->pPointCloud_->points[ii] << std::endl; 
    }*/

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapVoxelGridFilterActive<PointT>::Clear()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
}

template<typename PointT>
void PointCloudMapVoxelGridFilterActive<PointT>::OnMapChange()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxelGridFilterActive<PointT>::OnMapChange() - point cloud map reset *** " << std::endl;
        PointCloudMap<PointT>::Clear();
    }
}

template<>
void PointCloudMapVoxelGridFilterActive<pcl::PointSurfelSegment>::OnMapChange()
{
    typedef pcl::PointSurfelSegment PointT;
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef std::unordered_map<uint32_t, typename PointCloudMap<PointT>::PointCloudT> MapKfidCloud;
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    
    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxelGridFilterActive<PointT>::OnMapChange() - point cloud map reset *** " << std::endl;
        PointCloudMap<PointT>::Clear();
    }
    
    if(this->pPointCloudMapParameters_->bCloudDeformationOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxelGridFilterActive<PointT>::OnMapChange() - point cloud KF adjustment" << std::endl;
        
        this->UpdateMap(); // update the pointcloud 
        
        // KF Adjustment of the map 
        //      * iterate over all points of the map:
        //          - group them in pairs (KFID, vector of points with kfid==KFID)  (?couldn't we maintain this when we integrate the new sensed cloud in AM?)        
        //      * correct each group of KF points according to the new position of the corresponding KF 
        //      * re-integrate all the group of KF points in the new map
        
        MapKfidCloud mapKfidToPointCloud;        
        for(size_t ii=0,iiEnd=this->pPointCloud_->size(); ii<iiEnd; ii++)
        {
            const pcl::PointSurfelSegment& point = this->pPointCloud_->points[ii];
            mapKfidToPointCloud[point.kfid].points.push_back(point); 
        }
        
        // once recollected all the points, let's clear the map 
        PointCloudMap<PointT>::Clear();
        
        PointCloudT kfCloudWorldNew;
        const cv::Mat identity = cv::Mat::eye(4,4,CV_32F);
        MapKfidCloud::iterator it=mapKfidToPointCloud.begin(), itEnd=mapKfidToPointCloud.end();
        for(;it!=itEnd; it++)
        {
            const uint32_t kfid = it->first;
            PointCloudT& kfCloudWorld = it->second; 
            
            typename PointCloudKeyFrameT::Ptr pcKF = mapKfidPointCloudKeyFrame_[kfid];
            if(!pcKF->bIsValid) continue; 
            
            KeyFramePtr pKF = pcKF->pKF;
            if(pKF->isBad()) continue;
            
            // let's correct the cloud 
            const Sophus::SE3f& TwcIntegration = pcKF->TwcIntegration; // pose at the last time of integration 
            
            // cv::Mat TcwIntegration = cv::Mat::eye(4,4,CV_32F);
            // // invert by taking into account the structure of the homogeneous transformation matrix
            // cv::Mat RcwIntegration =  TwcIntegration.rowRange(0,3).colRange(0,3).t();
            // cv::Mat tcwIntegration = -RcwIntegration*TwcIntegration.rowRange(0,3).col(3);
            // RcwIntegration.copyTo(TcwIntegration.rowRange(0,3).colRange(0,3));
            // tcwIntegration.copyTo(TcwIntegration.rowRange(0,3).col(3));
            Sophus::SE3f TcwIntegration = TwcIntegration.inverse();
            
            Sophus::SE3f TwcNew = pKF->GetPoseInverse();  // new corrected pose 
            
            // let's compute the correction transformation 
            //cv::Mat Twnwo= TwcNew * TwcIntegration.inv(); // from world old to world new             
            Sophus::SE3f Twnwo = TwcNew * TcwIntegration; // from world old to world new 
            
            // check if the transformation is "big" enough otherwise do not re-transform the cloud 
            //double norm = cv::norm(Twnwo - identity);
            double norm = (Twnwo.matrix3x4() - Sophus::SE3f().matrix3x4()).norm();
            std::cout << "kfid: " << kfid << ", norm: " << norm << std::endl;  
            if( norm > kNormThresholdForEqualMatrices)
            {            
                this->TransformCameraCloudInWorldFrame(kfCloudWorld, Converter::toIsometry3d(Twnwo), kfCloudWorldNew);   
            }
            else
            {
                kfCloudWorldNew.swap(kfCloudWorld);
            }
            
            // update integration pose  
            pcKF->TwcIntegration = TwcNew;
            
            // reintegrate the cloud 
            *(this->pPointCloud_) += kfCloudWorldNew; // push the corrected cloud             
        }
        
        /// < update timestamp !
        //this->UpdateMapTimestamp(); 
        
        this->UpdateMap(); // filter and update timestamp 
         
    }
}



} //namespace PLVS2
