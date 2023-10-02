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

#ifndef POINTCLOUD_MAP_VOXBLOX_H
#define POINTCLOUD_MAP_VOXBLOX_H 

#include "PointCloudMap.h" 

namespace voxblox
{
    class TsdfServer;
}

namespace PLVS2
{

///	\class PointCloudMapVoxblox
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using voxblox 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapVoxblox : public PointCloudMap<PointT>
{
public:

    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;

public: 
        
    static std::string skIntegrationMethod;    

public:

    PointCloudMapVoxblox(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void InsertCloud(typename PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, double max_range);
    
    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData);

    int UpdateMap();
    void Clear();
    
    void OnMapChange();
    
    void SaveMap(const std::string& filename);
    
    bool LoadMap(const std::string& filename);    

protected:

    std::shared_ptr<voxblox::TsdfServer> pTsdfServer_;
     
};



#if !USE_NORMALS

template class PointCloudMapVoxblox<pcl::PointXYZRGBA>;

#else

template class PointCloudMapVoxblox<pcl::PointXYZRGBNormal>;

template class PointCloudMapVoxblox<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
