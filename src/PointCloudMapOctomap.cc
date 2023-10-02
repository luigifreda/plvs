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

#include "PointCloudMapOctomap.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>


#include "ColorOctomapServer.h"

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"


namespace PLVS
{

template<typename PointT>
const double PointCloudMapOctomap<PointT>::kMinResForApplyingLocalFilter = 0.005; // [m]

template<typename PointT>
const double PointCloudMapOctomap<PointT>::kDownsampleResFactor = 0.8;    

template<typename PointT>
PointCloudMapOctomap<PointT>::PointCloudMapOctomap(double resolution_in, double max_range) : PointCloudMap<PointT>(resolution_in)
{
    ColorOctomapParameters octomap_params;
    octomap_params.resolution = resolution_in;
    octomap_params.sensor_max_range = max_range;
    pColorOctomapServer_ = std::make_shared<ColorOctomapServer<PointT> >(octomap_params);
    
    double resolutionLocalFilter = this->kDownsampleResFactor * resolution_in;
    bApplyLocalFilter_ = resolutionLocalFilter >= this->kMinResForApplyingLocalFilter;
    localVoxelFilter_.setLeafSize(resolutionLocalFilter, resolutionLocalFilter, resolutionLocalFilter);
}

template<typename PointT>
void PointCloudMapOctomap<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    
    if(bApplyLocalFilter_) 
    {
        std::cout << "local filtering before octomap " << std::endl; 
        typename PointCloudMap<PointT>::PointCloudT::Ptr cloud_filtered(new PointCloudT());    
        this->localVoxelFilter_.setInputCloud(cloud_world);
        this->localVoxelFilter_.filter(*cloud_filtered);   
        pColorOctomapServer_->InsertCloudCallback(cloud_filtered, octomap::point3d(x_sensor_origin, y_sensor_origin, z_sensor_origin), max_range);        
    }
    else
    {
        pColorOctomapServer_->InsertCloudCallback(cloud_world, octomap::point3d(x_sensor_origin, y_sensor_origin, z_sensor_origin), max_range);   
    }

}

template<typename PointT>
void PointCloudMapOctomap<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    assert(pData->type == PointCloudMapInput<PointT>::kPointCloud);

    cv::Mat Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc; 

    const double x_sensor_origin = Twc.at<float>(0, 3);
    const double y_sensor_origin = Twc.at<float>(1, 3);
    const double z_sensor_origin = Twc.at<float>(2, 3);

    typename PointCloudT::Ptr pCloudWorld(new PointCloudT);
    this->TransformCameraCloudInWorldFrame(pData->pPointCloudKeyFrame->pCloudCamera, Twc, pCloudWorld);
    
    if(bApplyLocalFilter_) 
    {
        std::cout << "local filtering before octomap " << std::endl; 
        typename PointCloudMap<PointT>::PointCloudT::Ptr pCloudFiltered(new PointCloudT());    
        this->localVoxelFilter_.setInputCloud(pCloudWorld);
        this->localVoxelFilter_.filter(*pCloudFiltered);   
        pColorOctomapServer_->InsertCloudCallback(pCloudFiltered, octomap::point3d(x_sensor_origin, y_sensor_origin, z_sensor_origin), pData->maxRange);        
    }
    else
    {
        pColorOctomapServer_->InsertCloudCallback(pCloudWorld, octomap::point3d(x_sensor_origin, y_sensor_origin, z_sensor_origin), pData->maxRange);   
    }    

    //pColorOctomapServer_->InsertCloudCallback(pCloudWorld, octomap::point3d(x_sensor_origin, y_sensor_origin, z_sensor_origin), pData->maxRange);
}

template<typename PointT>
int PointCloudMapOctomap<PointT>::UpdateMap()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < update map 
    // map reset managed by pColorOctomapServer_
    pColorOctomapServer_->GetOccupiedPointCloud(this->pPointCloud_);

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapOctomap<PointT>::Clear()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();

    pColorOctomapServer_->Reset();
}

template<typename PointT>
void PointCloudMapOctomap<PointT>::OnMapChange()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    if (this->bResetOnSparseMapChange_)
    {
        std::cout << "PointCloudMapOctomap<PointT>::OnMapChange() - octomap reset *** " << std::endl;
        pColorOctomapServer_->Reset();
    }
}


} //namespace PLVS
