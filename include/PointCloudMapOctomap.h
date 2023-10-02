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

#ifndef POINTCLOUD_MAP_OCTOMAP_H
#define POINTCLOUD_MAP_OCTOMAP_H 

#include "PointCloudMap.h" 

#include <pcl/filters/voxel_grid.h>

namespace PLVS2
{

template<typename PointT>
class ColorOctomapServer;


///	\class PointCloudMapOctomap
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using an octomap 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapOctomap : public PointCloudMap<PointT>
{
public:
    
    static const double kMinResForApplyingLocalFilter;
    static const double kDownsampleResFactor;    

    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;

public:

    PointCloudMapOctomap(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void InsertCloud(typename PointCloudT::ConstPtr cloud_world, double x_sensor_origin, double y_sensor_origin, double z_sensor_origin, double max_range);

    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData);
    
    int UpdateMap();
    void Clear();
    
    void OnMapChange();

protected:

    std::shared_ptr<ColorOctomapServer<PointT> > pColorOctomapServer_;
    pcl::VoxelGrid<PointT> localVoxelFilter_;
    
    bool bApplyLocalFilter_; 
};




#if !USE_NORMALS

template class PointCloudMapOctomap<pcl::PointXYZRGBA>;

#else

template class PointCloudMapOctomap<pcl::PointXYZRGBNormal>;

template class PointCloudMapOctomap<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
