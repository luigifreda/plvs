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

#ifndef POINTCLOUD_ATLAS_H
#define POINTCLOUD_ATLAS_H

#include "Atlas.h"
#include "Pointers.h"

#include <opencv2/core/core.hpp>

#include "PointDefinitions.h"
#include "PointCloudKeyFrame.h"
#include "PointCloudMap.h"
#include "PointCloudMapTypes.h"

#include <set>
#include <mutex>


namespace PLVS2
{
    
class Map;

///	\class PointCloudAtlas
///	\author Luigi Freda
///	\brief A class for managing different PointCloudMap objects (each one associated to new Map object)
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudAtlas
{
public:

    typedef typename pcl::PointCloud<PointT> PointCloudT;
        
    using PointCloudMapType = PointCloudMapTypes::PointCloudMapType;
    
public:
    PointCloudAtlas(Atlas* atlas, const std::shared_ptr<PointCloudMapParameters> pPointCloudParameters);
    //PointCloudAtlas(int initKFid); // When its initialization the first map is created
    ~PointCloudAtlas();
    
    void CreateNewMap(Map* pMap);
    void ChangeMap(Map* pMap);

    
    // Method for change components in the current map
    //void AddKeyFrame(KeyFramePtr pKF);
    void InsertKeyFrame(typename PointCloudKeyFrame<PointT>::Ptr pcKeyFrame); 

    //TODO: Luigi add new camera support 
    //void AddCamera(GeometricCamera* pCam);

    // Method for get data in current map
    //std::vector<typename PointCloudKeyFrame<PointT>::Ptr> GetCurrentMapKeyFrames();
    typename PointCloudMap<PointT>::Ptr GetCurrentMap();

    std::vector<typename PointCloudMap<PointT>::Ptr> GetAllMaps();

    int CountMaps();

    void clearMap(Map* pMap); // connected to Atlas::clearMap()

    void clearAtlas(); // connected to Atlas::clearAtlas()

    //void SetMapBad(Map* pMap); // not needed since for each pointcloud map we are using the corresponding mpMap->IsBad()
    void RemoveBadMaps(); 
        
public: 
        
    typename PointCloudMap<PointT>::Ptr GetCorrespondingPointCloudMap(Map* pMap);
    
    typename PointCloudMap<PointT>::Ptr NewPointCloudMap(Map* pMap);
    
protected:

    std::shared_ptr<PointCloudMapParameters> pPointCloudMapParameters_;
   
    std::set<typename PointCloudMap<PointT>::Ptr> mspMaps;
    std::set<typename PointCloudMap<PointT>::Ptr> mspBadMaps;
    
    std::map<Map*, typename PointCloudMap<PointT>::Ptr> mMapTable;
    
    typename PointCloudMap<PointT>::Ptr mpCurrentMap;

    //Pinhole testCam;
    std::recursive_mutex mMutexPointCloudAtlas;

    //unsigned long int mnLastInitKFidMap;
    
    Atlas* mpAtlas;

}; // class PointCloudAtlas

} // namespace PLVS2

#endif // ATLAS_H
