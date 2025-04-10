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

#ifndef POINTCLOUD_MAP_OCTREE_PC_H
#define POINTCLOUD_MAP_OCTREE_PC_H 

#include "PointCloudMap.h" 

#include "OctreePointCloudCentroid.h"
#include <open_chisel/camera/PinholeCamera.h>


namespace PLVS2
{

///	\class PointCloudMapOctreePointCloud
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using a pcl octree with point confidence counter 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapOctreePointCloud : public PointCloudMap<PointT>
{
public:

    static const int kDefaultPointCounterThreshold;
    static const int kFactorCarvingThreshold; 
    static const float kMinCarvingThreshold; 
    static const int kFactorSigmaZ; 
    static const std::uint64_t kDeltaTimeForCleaningUnstablePointsUs; // [microseconds]
    
    enum PropertyType {kPointCounterThreshold=0, kNone};
    
public:
        
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef PointCloudKeyFrame<PointT> PointCloudKeyFrameT;       

    typedef typename pcl::octree::OctreePointCloudCentroid<PointT> OctreeType;
    //typedef typename pcl::octree::OctreePointCloud<PointT> OctreeType;
    //typedef typename pcl::octree::OctreePointCloudSinglePoint<PointT> OctreeType;

public:

    PointCloudMapOctreePointCloud(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);
    
    void SetDepthCameraModel(const PointCloudCamParams& params);
    
    // this integrates the input cloud in the octree by suitably averaging point fields (e.g. color, normals, label, etc...)
    void InsertCloud(typename PointCloudT::ConstPtr cloud_world);
    
    // this inserts the input cloud in the octree but forces each inserted octree point 
    // to match some specific fields of the corresponding input point (e.g. label_confidence ) without averaging.
    // this is used with cloud KF adjustment
    void ReinsertCloud(typename PointCloudT::ConstPtr cloud_world);
    
    //void InsertCloudWithDepthOld(typename PointCloudT::ConstPtr cloud_world, const cv::Mat& Twc, const cv::Mat& depthImage, double max_range);
    
    void InsertCloudWithDepthOld(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc);
    void InsertCloudWithDepthNoSegm(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc);
    void InsertCloudWithDepthSegm(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc);
    
    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData);
        
    int UpdateMap();
    int UpdateMapNoSegmNoRemoveUnstable();
    int UpdateMapNoSegm();
    int UpdateMapSegm();
    
    void Clear();
    
    void OnMapChange();
    
    
    bool LoadMap(const std::string& filename);

public:
    
    void SetIntProperty(unsigned int property, int val);

protected:

    chisel::PinholeCamera depthCameraModel_;
    
    OctreeType octree_;
    
    pcl::VoxelGrid<PointT> voxelFilter_;
    
    //int nPointCounterThreshold_;
    float carvingThreshold_; 
    
    std::unordered_map<uint32_t, typename PointCloudKeyFrameT::Ptr> mapKfidPointCloudKeyFrame_;    
};



#if !USE_NORMALS

template class PointCloudMapOctreePointCloud<pcl::PointXYZRGBA>;

#else

template class PointCloudMapOctreePointCloud<pcl::PointXYZRGBNormal>;

template class PointCloudMapOctreePointCloud<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
