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

#include "PointCloudAtlas.h"
#include "Map.h"

#include "PointCloudMapFastFusion.h"
#include "PointCloudMapVoxblox.h"
#include "PointCloudMapChisel.h"
#include "PointCloudMapOctreePointCloud.h"
#include "PointCloudMapOctomap.h"
#include "PointCloudMapVoxelGridFilter.h"
#include "Utils.h"

namespace PLVS2
{

template<typename PointT>
PointCloudAtlas<PointT>::PointCloudAtlas(Atlas* atlas, const std::shared_ptr<PointCloudMapParameters> pPointCloudParameters): 
    mpAtlas(atlas), pPointCloudMapParameters_(pPointCloudParameters)
{
    mpCurrentMap = static_cast<typename PointCloudMap<PointT>::Ptr>(NULL);
    Map* pMap = atlas->GetCurrentMap();
    CreateNewMap(pMap);
}

//template<typename PointT>
//PointCloudAtlas<PointT>::PointCloudAtlas(int initKFid): mnLastInitKFidMap(initKFid)
//{
//    mpCurrentMap = static_cast<typename PointCloudMap<PointT>::Ptr>(NULL);
////    CreateNewMap();
//}

template<typename PointT>
PointCloudAtlas<PointT>::~PointCloudAtlas()
{
//    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
//    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
//    {
//        Map* pMi = *it;
//
//        if(pMi)
//        {
//            delete pMi;
//            pMi = static_cast<Map*>(NULL);
//
//            it = mspMaps.erase(it);
//        }
//        else
//            ++it;
//
//    }
}

template<typename PointT>
void PointCloudAtlas<PointT>::CreateNewMap(Map* pMap)
{
    MSG_ASSERT(pMap, "Map should be non-NULL!");
    
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    cout << "Creation of new point cloud map with id: " << pMap->GetId() << endl;
    
    if(mpCurrentMap)
    {
        cout << "Exits current map " << endl;
//        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
//            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        cout << "Saved map with ID: " << mpCurrentMap->GetId() << endl;

    }
    //cout << "Creation of new point cloud map with last KF id: " << mnLastInitKFidMap << endl;

    mpCurrentMap = NewPointCloudMap(pMap);
    mpCurrentMap->SetPointCloudMapParameters(pPointCloudMapParameters_);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

template<typename PointT>
typename PointCloudMap<PointT>::Ptr PointCloudAtlas<PointT>::NewPointCloudMap(Map* pMap)
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas); 
    
    typename PointCloudMap<PointT>::Ptr pPointCloudMap;
    
    std::string& pointCloudMapStringType = pPointCloudMapParameters_->pointCloudMapStringType;
    auto& pointCloudMapType = pPointCloudMapParameters_->pointCloudMapType;
    
    // create a new point cloud map 
    if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kVoxelGrid])
    {
        //pPointCloudMap = std::make_shared<PointCloudMapVoxelGridFilter<PointT> >(pMap, pPointCloudMapParameters_);
        pPointCloudMap = std::make_shared<PointCloudMapVoxelGridFilterActive<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kVoxelGrid;
    }
    else if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kOctomap])
    {
        pPointCloudMap = std::make_shared<PointCloudMapOctomap<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kOctomap;
    }
    else if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kOctreePoint])
    {
        pPointCloudMap = std::make_shared<PointCloudMapOctreePointCloud<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kOctreePoint;
    }
    else if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kChisel])
    {
        pPointCloudMap = std::make_shared<PointCloudMapChisel<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kChisel;
    }
    else if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kFastFusion])
    {
        pPointCloudMap = std::make_shared<PointCloudMapFastFusion<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kFastFusion;
    }
    else if (pointCloudMapStringType == PointCloudMapTypes::kPointCloudMapTypeStrings[PointCloudMapTypes::kVoxblox])
    {
        pPointCloudMap = std::make_shared<PointCloudMapVoxblox<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kVoxblox;
    }    
    else
    {
        // default 
        pPointCloudMap = std::make_shared<PointCloudMapVoxelGridFilter<PointT> >(pMap, pPointCloudMapParameters_);
        pointCloudMapType = PointCloudMapTypes::kVoxelGrid;
        
        std::cout << "PointCloudMapping::PointCloudMapping() - selecting default point map PointCloudMapVoxelGridFilter " << std::endl; 
    }    
    
    mMapTable[pMap] = pPointCloudMap;
    return pPointCloudMap;
}

template<typename PointT>
//void PointCloudAtlas<PointT>::ChangeMap(typename PointCloudMap<PointT>::Ptr pMap)
void PointCloudAtlas<PointT>::ChangeMap(Map* pMap)
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    cout << "Change to point cloud map with id: " << pMap->GetId() << endl;
    
    if(mpCurrentMap)
    {
        mpCurrentMap->SetStoredMap();
    }

    auto it = mMapTable.find(pMap);
    if(it!=mMapTable.end())
    {
        mpCurrentMap = it->second;
        mpCurrentMap->SetCurrentMap();
    }
    else
    {
        MSG_ASSERT(false,"We must have a correspondence here!");   
    }
}
    
template<typename PointT>
vector<typename PointCloudMap<PointT>::Ptr> PointCloudAtlas<PointT>::GetAllMaps()
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    struct compFunctor
    {
        inline bool operator()(typename PointCloudMap<PointT>::Ptr elem1 ,typename PointCloudMap<PointT>::Ptr elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<typename PointCloudMap<PointT>::Ptr> vMaps(mspMaps.begin(),mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

template<typename PointT>
int PointCloudAtlas<PointT>::CountMaps()
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    return mspMaps.size();
}

template<typename PointT>
void PointCloudAtlas<PointT>::clearMap(Map* pMap)
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    auto pPointCloudMap = GetCorrespondingPointCloudMap(pMap);
    pPointCloudMap->Clear();
}

template<typename PointT>
void PointCloudAtlas<PointT>::clearAtlas()
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    for(auto it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
    {
        (*it)->Clear();
    }
    mspMaps.clear();     
    mpCurrentMap = static_cast<typename PointCloudMap<PointT>::Ptr>(NULL);
    //mnLastInitKFidMap = 0;
}

template<typename PointT>
typename PointCloudMap<PointT>::Ptr PointCloudAtlas<PointT>::GetCurrentMap()
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
    if(!mpCurrentMap)
    {
        Map* pMap = mpAtlas->GetCurrentMap(); 
        CreateNewMap(pMap);
    }
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

// template<typename PointT>
// void PointCloudAtlas<PointT>::SetMapBad(Map* pMap)
// {
//    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);
//    auto pPointCloudMap = GetCorrespondingPointCloudMap(pMap);    
   
//    mspMaps.erase(pPointCloudMap);
//    pPointCloudMap->SetBad();

//    mspBadMaps.insert(pPointCloudMap);
// }

template<typename PointT>
void PointCloudAtlas<PointT>::RemoveBadMaps()
{
    unique_lock<recursive_mutex> lock(mMutexPointCloudAtlas);    
    /*for(Map* pMap : mspBadMaps)
    {
        delete pMap;
        pMap = static_cast<Map*>(NULL);
    }*/
    mspBadMaps.clear();
}

template<typename PointT>
typename PointCloudMap<PointT>::Ptr PointCloudAtlas<PointT>::GetCorrespondingPointCloudMap(Map* pMap)
{
    auto it = mMapTable.find(pMap);
    if(it!=mMapTable.end())
    {
        return it->second;
    }
    else
    {
        MSG_ASSERT(false,"We must have a correspondence here!")
        return static_cast<typename PointCloudMap<PointT>::Ptr>(NULL);
    }    
}

#if !USE_NORMALS

template class PointCloudAtlas<pcl::PointXYZRGBA>;

#else

template class PointCloudAtlas<pcl::PointXYZRGBNormal>;

template class PointCloudAtlas<pcl::PointSurfelSegment>;

#endif

} //namespace PLVS2
