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

#ifndef POINTCLOUD_MAP_CHISEL_H
#define POINTCLOUD_MAP_CHISEL_H 

#include "PointCloudMap.h" 


namespace chisel
{
class PinholeCamera;
}

namespace chisel_server
{
class ChiselServer;
struct ChiselServerParams;
}

namespace PLVS2
{

///	\class PointCloudMapOctreePointCloud
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using open chisel lib 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapChisel : public PointCloudMap<PointT>
{
public:

    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef PointCloudKeyFrame<PointT> PointCloudKeyFrameT;       

public:

    PointCloudMapChisel(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void SetDepthCameraModel(const PointCloudCamParams& params);
    void SetColorCameraModel(const PointCloudCamParams& params);
        
    void InsertCloud(typename PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, double max_range);
    
    void InsertCloudWithDepth(typename PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, const cv::Mat& depthImage, double max_range);
    
    void InsertDepthScanColor(const cv::Mat& depthImage, const cv::Mat& colorImage, const Sophus::SE3f& Twc, boost::uint64_t timestamp);
    
    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData);

    int UpdateMap();
    void Clear();
    
    void OnMapChange();
    
    void SaveMap(const std::string& filename);
    
    bool LoadMap(const std::string& filename);    

protected:

    std::shared_ptr<chisel_server::ChiselServer> pChiselServer_;
    std::shared_ptr<chisel_server::ChiselServerParams> pChiselServerParams_;
    
    //std::unordered_map<uint32_t, typename PointCloudKeyFrameT::Ptr> mapKfidPointCloudKeyFrame_;        
    std::map<uint32_t, typename PointCloudKeyFrameT::Ptr> mapKfidPointCloudKeyFrame_; // ordered map since we use its order in the KF adjustment 

};




#if !USE_NORMALS

template class PointCloudMapChisel<pcl::PointXYZRGBA>;

#else

template class PointCloudMapChisel<pcl::PointXYZRGBNormal>;

template class PointCloudMapChisel<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
