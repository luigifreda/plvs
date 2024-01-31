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


#include "Map.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include<mutex>

namespace PLVS2
{

long unsigned int Map::nNextId=0;

Map::Map():mnInitKFid(0), mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFramePtr>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFramePtr>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    std::cout << "~Map(): destroyed id " << mnId << std::endl;
        
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all lines from memory
    mspMapLines.clear();

    //TODO: erase all objects from memory
    mspMapObjects.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpReferenceMapLines.clear();
    mvpReferenceMapObjects.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
        cout << "First KF:" << pKF->mnId << "; Map " << pKF->GetMap()->mnId << " init KF:" << mnInitKFid << endl;        
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(const MapPointPtr& pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
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

void Map::EraseKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFramePtr> vpKFs = vector<KeyFramePtr>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    /// < TODO: This only erase the pointer.
    // Delete the KeyFrame
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

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}
long unsigned int Map::GetInitKFidNoLock() const
{
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFramePtr Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPointPtr>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;
//
    for(set<KeyFramePtr>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFramePtr pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
//        DeletePtr(*sit);
    }

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
    mspMapObjects.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    //mnLastLoopKFid = 0;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpReferenceMapLines.clear();
    mvpReferenceMapObjects.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

void Map::printStatistics()
{
    unique_lock<mutex> lock(mMutexMap);

    std::cout << "********************" << std::endl; 
    std::cout << "Map " << mnId << std::endl; 
    
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
    std::cout << "********************" << std::endl;     
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(set<KeyFramePtr>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFramePtr pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPointPtr>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPointPtr pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    
    // lines
    for(set<MapLinePtr>::iterator sit=mspMapLines.begin(); sit!=mspMapLines.end(); sit++)
    {
        MapLinePtr pML = *sit;
        
        Eigen::Vector3f PosStart, PosEnd;
        pML->GetWorldEndPoints(PosStart, PosEnd);
        pML->SetWorldEndPoints(s*Ryw*PosStart+tyw, s*Ryw*PosEnd+tyw);
        pML->UpdateNormalAndDepth();
    }    

    // objects 
    for(set<MapObjectPtr>::iterator sit=mspMapObjects.begin(); sit!=mspMapObjects.end(); sit++)
    {
        MapObjectPtr pMO = *sit;
        
        Eigen::Matrix3f Rwo;
        Eigen::Vector3f two; 
        const double swo = pMO->GetScale();
        Rwo = pMO->GetInverseRotation();
        two = pMO->GetInverseTranslation();
                          
        pMO->SetSim3InversePose(Ryw*Rwo, s*Ryw*two+tyw, swo*s);  // Swo = [swo*Rwo, two; 0, 1]     
    }

    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::PrintEssentialGraph()
{
    //Print the essential graph
    vector<KeyFramePtr> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFramePtr pFirstKF;
    for(KeyFramePtr pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    cout << "KF: " << pFirstKF->mnId << endl;
    set<KeyFramePtr> spChilds = pFirstKF->GetChilds();
    vector<KeyFramePtr> vpChilds;
    vector<string> vstrHeader;
    for(KeyFramePtr pKFi : spChilds){
        vstrHeader.push_back("--");
        vpChilds.push_back(pKFi);
    }
    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        string strHeader = vstrHeader[i];
        KeyFramePtr pKFi = vpChilds[i];

        cout << strHeader << "KF: " << pKFi->mnId << endl;

        set<KeyFramePtr> spKFiChilds = pKFi->GetChilds();
        for(KeyFramePtr pKFj : spKFiChilds)
        {
            vpChilds.push_back(pKFj);
            vstrHeader.push_back(strHeader+"--");
        }
    }
    if (count == (mspKeyFrames.size()+10))
        cout << "CYCLE!!"    << endl;

    cout << "------------------" << endl << "End of the essential graph" << endl;
}

bool Map::CheckEssentialGraph(){
    vector<KeyFramePtr> vpOriginKFs = mvpKeyFrameOrigins;
    int count=0;
    cout << "Number of origin KFs: " << vpOriginKFs.size() << endl;
    KeyFramePtr pFirstKF;
    for(KeyFramePtr pKFi : vpOriginKFs)
    {
        if(!pFirstKF)
            pFirstKF = pKFi;
        else if(!pKFi->GetParent())
            pFirstKF = pKFi;
    }
    cout << "Checking if the first KF has parent" << endl;
    if(pFirstKF->GetParent())
    {
        cout << "First KF in the essential graph has a parent, which is not possible" << endl;
    }

    set<KeyFramePtr> spChilds = pFirstKF->GetChilds();
    vector<KeyFramePtr> vpChilds;
    vpChilds.reserve(mspKeyFrames.size());
    for(KeyFramePtr pKFi : spChilds)
        vpChilds.push_back(pKFi);

    for(int i=0; i<vpChilds.size() && count <= (mspKeyFrames.size()+10); ++i)
    {
        count++;
        KeyFramePtr pKFi = vpChilds[i];
        set<KeyFramePtr> spKFiChilds = pKFi->GetChilds();
        for(KeyFramePtr pKFj : spKFiChilds)
            vpChilds.push_back(pKFj);
    }

    cout << "count/tot" << count << "/" << mspKeyFrames.size() << endl;
    if (count != (mspKeyFrames.size()-1))
        return false;
    else
        return true;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::printReprojectionError(list<KeyFramePtr> &lpLocalWindowKFs, KeyFramePtr mpCurrentKF, string &name, string &name_folder)
{
    string path_imgs = "./" + name_folder + "/";
    for(KeyFramePtr pKFi : lpLocalWindowKFs)
    {
        //cout << "KF " << pKFi->mnId << endl;
        cv::Mat img_i = cv::imread(pKFi->mNameFile, cv::IMREAD_UNCHANGED);
        //cout << "Image -> " << img_i.cols << ", " << img_i.rows << endl;
        cv::cvtColor(img_i, img_i, cv::COLOR_GRAY2BGR);
        //cout << "Change of color in the image " << endl;

        vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
        int num_points = 0;
        for(int j=0; j<vpMPs.size(); ++j)
        {
            MapPointPtr pMPij = vpMPs[j];
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            cv::KeyPoint point_img = pKFi->mvKeysUn[j];
            cv::Point2f reproj_p;
            float u, v;
            bool bIsInImage = pKFi->ProjectPointUnDistort(pMPij, reproj_p, u, v);
            if(bIsInImage){
                //cout << "Reproj in the image" << endl;
                cv::circle(img_i, point_img.pt, 1/*point_img.octave*/, cv::Scalar(0, 255, 0));
                cv::line(img_i, point_img.pt, reproj_p, cv::Scalar(0, 0, 255));
                num_points++;
            }
            else
            {
                //cout << "Reproj out of the image" << endl;
                cv::circle(img_i, point_img.pt, point_img.octave, cv::Scalar(0, 0, 255));
            }

        }
        //cout << "Image painted" << endl;
        string filename_img = path_imgs +  "KF" + to_string(mpCurrentKF->mnId) + "_" + to_string(pKFi->mnId) +  name + "points" + to_string(num_points) + ".png";
        cv::imwrite(filename_img, img_i);
    }

}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPointPtr pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFramePtr, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFramePtr, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }

        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();
    for(MapPointPtr pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for(KeyFramePtr pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

#if 0
void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFramePtr>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPointPtr> mpMapPointId;
    for(MapPointPtr pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFramePtr> mpKeyFrameId;
    for(KeyFramePtr pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for(MapPointPtr pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for(KeyFramePtr pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }


    if(mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if(mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}
#endif 

// template<class Archive>
// void serialize(Archive &ar, const unsigned int version)
// {
//     ar & mnId;
//     ar & mnInitKFid;
//     ar & mnMaxKFid;
//     ar & mnBigChangeIdx;

//     // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
//     //ar & mspKeyFrames;
//     //ar & mspMapPoints;
//     ar & mvpBackupKeyFrames;
//     ar & mvpBackupMapPoints;

//     ar & mvBackupKeyFrameOriginsId;

//     ar & mnBackupKFinitialID;
//     ar & mnBackupKFlowerID;

//     ar & mbImuInitialized;
//     ar & mbIsInertial;
//     ar & mbIMU_BA1;
//     ar & mbIMU_BA2;
// }

template<class Archive>
void Map::serialize(Archive &ar, const unsigned int version)
{
    UNUSED_VAR(version);
    unique_lock<mutex> lock(mMutexMap);
    
    if (Archive::is_saving::value) 
    {
        std::cout << "Saving map " << mnId << std::endl; 
    }
    if (Archive::is_loading::value) 
    {
        std::cout << "Loading map " << mnId << std::endl; 
    }    
    
    ar & nNextId;
    ar & mnId;
    ar & mnInitKFid;
    ar & mnMaxKFid;
    ar & mnBigChangeIdx;
    
    ar & mspKeyFrames;
    ar & mspMapPoints;
    ar & mspMapLines;    
    
    // don't save mutexes
    ar &mspMapPoints;
    ar &mspMapLines;
    ar &mvpKeyFrameOrigins;
    ar &mspKeyFrames;
    ar &mvpReferenceMapPoints;
    ar &mvpReferenceMapLines;
    //ar &mnMaxKFid &mnBigChangeIdx;    

    //ar & mvpBackupKeyFrames;
    //ar & mvpBackupMapPoints;

    //ar & mvBackupKeyFrameOriginsId;

    //ar & mnBackupKFinitialID;
    //ar & mnBackupKFlowerID;
    ar & mpKFinitial;
    ar & mpKFlowerID;

    ar & mbImuInitialized;
    ar & mbIsInertial;
    ar & mbIMU_BA1;
    ar & mbIMU_BA2;

    //ar & mnLastLoopKFid;
    
    mbIsLoaded = true; 
}
template void Map::serialize(boost::archive::binary_iarchive &, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive &, const unsigned int);
template void Map::serialize(boost::archive::text_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::text_oarchive&, const unsigned int);

} //namespace PLVS2
