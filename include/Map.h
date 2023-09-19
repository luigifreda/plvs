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

#ifndef MAP_H
#define MAP_H

#include "KeyFrame.h"
#include <set>

#include <mutex>

#include "BoostArchiver.h"
#include "Pointers.h"

namespace PLVS
{

//class MapPoint;
//class KeyFrame;
//class MapLine;
//class MapObject;

/// < TODO: erase the pointers of MapPoint, MapLine, KeyFrame

class Map
{
public:
    Map();

    void AddKeyFrame(const KeyFramePtr& pKF);
    void EraseKeyFrame(const KeyFramePtr& pKF);
    
    void AddMapPoint(const MapPointPtr& pMP);
    void EraseMapPoint(const MapPointPtr& pMP);
    void SetReferenceMapPoints(const std::vector<MapPointPtr> &vpMPs);

    void AddMapLine(const MapLinePtr& pML);    
    void EraseMapLine(const MapLinePtr& pML);
    void SetReferenceMapLines(const std::vector<MapLinePtr> &vpMLs);
    
    void AddMapObject(const MapObjectPtr& pMObj);   
    void EraseMapObject(const MapObjectPtr& pMObj);    
    void SetReferenceMapObjects(const std::vector<MapObjectPtr> &vpMObjs);    
        
    void InformNewBigChange();
    int GetLastBigChangeIdx();
    
    std::vector<KeyFramePtr> GetAllKeyFrames();
    std::set<KeyFramePtr> GetSetKeyFrames();
    
    std::vector<MapPointPtr> GetAllMapPoints();
    std::vector<MapLinePtr> GetAllMapLines();
    std::vector<MapObjectPtr> GetAllMapObjects();    
    
    std::vector<MapPointPtr> GetReferenceMapPoints();
    void GetSetOfReferenceMapPoints(std::set<MapPointPtr>& setMPs);
    std::vector<MapLinePtr> GetReferenceMapLines();
    void GetSetOfReferenceMapLines(std::set<MapLinePtr>& setMLs);    
    std::vector<MapObjectPtr> GetReferenceMapObjects();       
    void GetSetOfReferenceMapObjects(std::set<MapObjectPtr>& setMObjs);    

    long unsigned int MapPointsInMap();
    long unsigned int MapLinesInMap();
    long unsigned int MapObjectsInMap();    
    
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();
    
    void clear();

    void printStatistics();
        
    vector<KeyFramePtr> mvpKeyFrameOrigins;

    bool mnEnableLoopClosing;
    std::mutex mMutexLoopClosing;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // This avoid that two lines are created simultaneously in separate threads (id conflict)
    std::mutex mMutexLineCreation;
    
    // This avoid that two objects are created simultaneously in separate threads (id conflict)
    std::mutex mMutexObjectCreation;    
    
 private:
    // serialize is recommended to be private
    friend class boost::serialization::access;
    template < class Archive >
    void serialize(Archive& ar, const unsigned int version);

protected:
    std::set<MapPointPtr> mspMapPoints;
    std::set<MapLinePtr> mspMapLines;
    std::set<MapObjectPtr> mspMapObjects;    
    std::set<KeyFramePtr> mspKeyFrames;

    std::vector<MapPointPtr> mvpReferenceMapPoints;
    std::vector<MapLinePtr> mvpReferenceMapLines;
    std::vector<MapObjectPtr > mvpReferenceMapObjects;     

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;
    
    std::mutex mMutexMap;
};

} //namespace PLVS

#endif // MAP_H
