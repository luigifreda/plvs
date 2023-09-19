/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"

#include<mutex>

namespace PLVS
{

Map::Map():mnEnableLoopClosing(true),mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::EraseKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    /// < TODO: This only erase the pointer.
    // Delete the KeyFrame
}


void Map::AddMapPoint(const MapPointPtr& pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(const MapPointPtr& pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    /// < TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPointPtr> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::AddMapLine(const MapLinePtr& pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.insert(pML);    
}

void Map::EraseMapLine(const MapLinePtr& pML)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapLines.erase(pML);

    /// < TODO: This only erase the pointer.
    // Delete the MapLine
}

void Map::SetReferenceMapLines(const vector<MapLinePtr> &vpMLs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapLines = vpMLs;
}

void Map::AddMapObject(const MapObjectPtr& pMObj)
{    
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.insert(pMObj);        
}

void Map::EraseMapObject(const MapObjectPtr& pMObj)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.erase(pMObj);
}

void Map::SetReferenceMapObjects(const std::vector<MapObjectPtr > &vpMObjs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapObjects = vpMObjs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFramePtr> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFramePtr>(mspKeyFrames.begin(),mspKeyFrames.end());
}

set<KeyFramePtr> Map::GetSetKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames;
}

vector<MapPointPtr> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPointPtr>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapLinePtr> Map::GetAllMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapLinePtr>(mspMapLines.begin(),mspMapLines.end());
}

std::vector<MapObjectPtr > Map::GetAllMapObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapObjectPtr >(mspMapObjects.begin(),mspMapObjects.end());    
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::MapLinesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapLines.size();
}

long unsigned int Map::MapObjectsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapObjects.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPointPtr> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

void Map::GetSetOfReferenceMapPoints(std::set<MapPointPtr>& setMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    setMPs = std::set<MapPointPtr>(mvpReferenceMapPoints.begin(), mvpReferenceMapPoints.end());
}

vector<MapLinePtr> Map::GetReferenceMapLines()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapLines;
}

void Map::GetSetOfReferenceMapLines(std::set<MapLinePtr>& setMLs)
{
    unique_lock<mutex> lock(mMutexMap);
    setMLs = std::set<MapLinePtr>(mvpReferenceMapLines.begin(), mvpReferenceMapLines.end());
}

std::vector<MapObjectPtr > Map::GetReferenceMapObjects()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapObjects;    
}

void Map::GetSetOfReferenceMapObjects(std::set<MapObjectPtr >& setMObjs)
{
    unique_lock<mutex> lock(mMutexMap);
    setMObjs = std::set<MapObjectPtr >(mvpReferenceMapObjects.begin(), mvpReferenceMapObjects.end());
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    unique_lock<mutex> lock(mMutexMap);
        
    for(set<MapPointPtr>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
    {
        //delete *sit;
        DeletePtr(*sit);
    }

    for(set<MapLinePtr>::iterator sit=mspMapLines.begin(), send=mspMapLines.end(); sit!=send; sit++)
    {
        //delete *sit;
        DeletePtr(*sit);    
    }
    
    for(set<KeyFramePtr>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        //delete *sit;
        DeletePtr(*sit);          
    }
    
    for(set<MapObjectPtr>::iterator sit=mspMapObjects.begin(), send=mspMapObjects.end(); sit!=send; sit++)
    {
        //delete *sit;
        DeletePtr(*sit);
    }    

    mspMapPoints.clear();
    mspMapLines.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mspMapObjects.clear();
    
    mvpReferenceMapPoints.clear();
    mvpReferenceMapLines.clear();
    mvpKeyFrameOrigins.clear();
    mvpReferenceMapObjects.clear();
}

void Map::printStatistics()
{
    unique_lock<mutex> lock(mMutexMap);
    
    
    // check points 
    size_t numPoints = 0;
    size_t numBadPoints = 0; 
    size_t numNullPoints = 0; // just for checking 
    for(set<MapPointPtr>::iterator vit=mspMapPoints.begin(), vend=mspMapPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr pMP = *vit;
        if(!pMP) 
        {
            numNullPoints++;
            continue;
        }
        
        if(pMP->isBad()) 
        {
            numBadPoints++;
        }    
        else
        {
            numPoints++;
        }
    }    
    std::cout << "num points: " << numPoints << ", bad %: " << float(numBadPoints)/numPoints << ", null: " << numNullPoints << std::endl; 
    
    // check lines 
    size_t numLines = 0;
    size_t numBadLines = 0; 
    size_t numNullLines = 0; // just for checking 
    for(set<MapLinePtr>::iterator vit=mspMapLines.begin(), vend=mspMapLines.end(); vit!=vend; vit++)
    {
        MapLinePtr pML = *vit;
        if(!pML) 
        {
            numNullLines++;
            continue;
        }
        
        if(pML->isBad()) 
        {
            numBadLines++;
        }    
        else
        {
            numLines++;
        }
    }     
    std::cout << "num lines: " << numLines << ", bad %: " << float(numBadLines)/std::max(numLines,(size_t)1) << ", null: " << numNullLines << std::endl;     
    
    // check keyframes 
    size_t numKeyFrames = 0;
    size_t numBadKeyFrames = 0; 
    size_t numNullKeyFrames = 0; // just for checking 
    for(set<KeyFramePtr>::iterator vit=mspKeyFrames.begin(), vend=mspKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFramePtr pKF = *vit;
        if(!pKF) 
        {
            numNullKeyFrames++;
            continue;
        }
        
        if(pKF->isBad()) 
        {
            numBadKeyFrames++;
        }    
        else
        {
            numKeyFrames++;
        }
    }     
    std::cout << "num keyframes: " << numKeyFrames << ", bad %: " << float(numBadKeyFrames)/numKeyFrames << ", null: " << numNullKeyFrames << std::endl;      
}

template < class Archive >
void Map::serialize(Archive &ar, const unsigned int version) 
{
    UNUSED_VAR(version);
        
    unique_lock<mutex> lock(mMutexMap);
    
    // don't save mutexes
    ar &mspMapPoints;
    ar &mspMapLines;
    ar &mvpKeyFrameOrigins;
    ar &mspKeyFrames;
    ar &mvpReferenceMapPoints;
    ar &mvpReferenceMapLines;
    ar &mnMaxKFid &mnBigChangeIdx;
}
template void Map::serialize(boost::archive::binary_iarchive &,
                             const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive &,
                             const unsigned int);

} //namespace PLVS
