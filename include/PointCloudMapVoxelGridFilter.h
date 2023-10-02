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

#ifndef POINTCLOUD_MAP_VOXELGRIDFILTER_H
#define POINTCLOUD_MAP_VOXELGRIDFILTER_H 

#include "PointCloudMap.h" 
#include "VoxelGridCustom.h"

#include <mutex>
#include <memory>
#include <unordered_map>

namespace PLVS2
{

template<typename PointT>
class KeyFrameSearchTree;

///	\class PointCloudMapVoxelGridFilter
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using a voxel grid filter 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapVoxelGridFilter : public PointCloudMap<PointT>
{
public:
    
    static const int kDefaultPointCounterThreshold;    
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;

public:

    PointCloudMapVoxelGridFilter(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void InsertCloud(typename PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range);

    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData); 
    
    int UpdateMap();
    void Clear();
    
    void OnMapChange();

protected:

    pcl::VoxelGrid<PointT> voxel_;
};


///	\class PointCloudModel
///	\author Luigi Freda
///	\brief Class for managing active and inactive models 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapModel
{
public:
    
    typedef typename pcl::PointCloud<PointT> PointCloudT;
    typedef std::vector<uint32_t> PointList;  
    typedef std::shared_ptr<PointList> PointListPtr;
    
public:
    
    PointCloudMapModel();
    
    // for inactive model 
    void InsertCloud(uint32_t kfid, typename PointCloudT::Ptr pCloud);
    
    // for active model 
    void UpdateCloud(typename PointCloudT::Ptr pCloud);
        
protected:    

    std::unordered_map<uint32_t, PointListPtr> mapKfidToPointList_;
    typename PointCloudT::Ptr pPointCloud_; // active model
};



///	\class PointCloudMapVoxelGridFilterActive
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using a voxel grid filter 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapVoxelGridFilterActive : public PointCloudMap<PointT>
{
public:
    
    using PointCloudMap<PointT>::kNormThresholdForEqualMatrices;

    static const int kDefaultPointCounterThreshold;          
    static const float kKdtreeSearchRangeFactor; 
    
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef PointCloudKeyFrame<PointT> PointCloudKeyFrameT;    

public:

    PointCloudMapVoxelGridFilterActive(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void InsertCloud(typename PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range);

    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData); 
    
    int UpdateMap();
    void Clear();
    
    void OnMapChange();

protected:

    pcl::VoxelGridCustom<PointT> voxel_;
    
    std::set<KeyFramePtr> setActiveKFs_;
    std::set<uint32_t> setActiveIds_;    
    std::shared_ptr<KeyFrameSearchTree<pcl::PointXYZL> > pKfSearchTree_; 
    
    std::unordered_map<uint32_t, typename PointCloudKeyFrameT::Ptr> mapKfidPointCloudKeyFrame_;
      
};



/// < list here the types you want to use 

#if !USE_NORMALS

template class PointCloudMapVoxelGridFilter<pcl::PointXYZRGBA>;

template class PointCloudMapVoxelGridFilterActive<pcl::PointXYZRGBA>;


#else

template class PointCloudMapVoxelGridFilter<pcl::PointXYZRGBNormal>;
template class PointCloudMapVoxelGridFilter<pcl::PointSurfelSegment>;

template class PointCloudMapVoxelGridFilterActive<pcl::PointXYZRGBNormal>;
template class PointCloudMapVoxelGridFilterActive<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
