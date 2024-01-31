/*
 * This file is part of PLVS.

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

/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAP_H
#define MAP_H

//#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include "BoostArchiver.h"
#include "Pointers.h"

namespace PLVS2
{

//class MapPoint;
//class KeyFrame;
class Atlas;
class KeyFrameDatabase;
class GeometricCamera;

class Map
{
    friend class Atlas; 
    
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

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

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    long unsigned int GetInitKFidNoLock() const;
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFramePtr GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();
    
    bool isLoaded() const { return mbIsLoaded; }

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFramePtr>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFramePtr> &lpLocalWindowKFs, KeyFramePtr mpCurrentKF, string &name, string &name_folder);

    vector<KeyFramePtr> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFramePtr mpFirstRegionKF;

    void printStatistics();

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // This avoid that two lines are created simultaneously in separate threads (id conflict)
    std::mutex mMutexLineCreation;
    
    // This avoid that two objects are created simultaneously in separate threads (id conflict)
    std::mutex mMutexObjectCreation;    

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    std::set<MapPointPtr> mspMapPoints;
    std::set<MapLinePtr> mspMapLines;
    std::set<MapObjectPtr> mspMapObjects;    
    std::set<KeyFramePtr> mspKeyFrames;

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<MapPointPtr> mvpBackupMapPoints;
    std::vector<KeyFramePtr> mvpBackupKeyFrames;

    KeyFramePtr mpKFinitial;
    KeyFramePtr mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPointPtr> mvpReferenceMapPoints;
    std::vector<MapLinePtr> mvpReferenceMapLines;
    std::vector<MapObjectPtr > mvpReferenceMapObjects;     


    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;
    
    bool mbIsLoaded = false; // is it a map loaded from mapping storage?

    // Mutex
    std::mutex mMutexMap;

};

} //namespace PLVS2

#endif // MAP_H
