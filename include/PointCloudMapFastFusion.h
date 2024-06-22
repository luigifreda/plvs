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

#ifndef POINTCLOUD_MAP_FASTFUSION_H
#define POINTCLOUD_MAP_FASTFUSION_H 

#include "PointCloudMap.h" 


#ifdef USE_FASTFUSION
class FusionMipMapCPU;
class MeshInterleaved; 
class PointerMeshDraw; 
#endif 


namespace PLVS2
{

///	\class PointCloudMapFastFusion
///	\author Luigi Freda
///	\brief Class for merging/managing point clouds by using fastfusion lib 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudMapFastFusion : public PointCloudMap<PointT>
{
public:

    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    
    enum PropertyType {kUseCarve=0, kNone};

public:

    PointCloudMapFastFusion(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params);

    void Init();
        
    void SetDepthCameraModel(const PointCloudCamParams& params);
    void SetColorCameraModel(const PointCloudCamParams& params);
            
    void InsertData(typename PointCloudMapInput<PointT>::Ptr pData);

    int UpdateMap();
    void Clear();
    
    void OnMapChange();
    
    void GetMapWithTimeout(typename PointCloudT::Ptr& pCloud, typename PointCloudT::Ptr& pCloudUnstable, 
                                   std::vector<unsigned int>& faces, const std::chrono::milliseconds& timeout, bool copyUnstable = false);   
    
    void SaveMap(const std::string& filename);
    
    bool LoadMap(const std::string& filename);
    
protected:

    PointCloudCamParams depthCameraModel_;
    PointCloudCamParams colorCameraModel_;

#ifdef USE_FASTFUSION    
    std::shared_ptr<FusionMipMapCPU> pFusion_; 
    std::shared_ptr<MeshInterleaved> pCurrentMeshInterleaved_; 
#endif
    
    bool useColor;
    
    bool threadMeshing;
    bool performIncrementalMeshing;
    int depthConstistencyChecks;
    
    float scale;
    float distanceThreshold;
    
};





#if !USE_NORMALS

template class PointCloudMapFastFusion<pcl::PointXYZRGBA>;

#else

template class PointCloudMapFastFusion<pcl::PointXYZRGBNormal>;

template class PointCloudMapFastFusion<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2



#endif // POINTCLOUD_ENGINE_H
