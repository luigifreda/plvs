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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"
#include "Tracking.h"
#include "Geom2DUtils.h"

#include<mutex>

#define ENABLE_CHANGES_SELFCHILD_ISSUE 1
#define ENABLE_NEW_CHANGES 1
#define DEBUG_DESTRUCTOR 1
  

namespace PLVS
{

long unsigned int KeyFrame::nNextId=0;

const std::vector<size_t> KeyFrame::kEmptyVecSizet = std::vector<size_t>();

const float KeyFrame::kDeltaTheta = Frame::kDeltaTheta;
const float KeyFrame::kDeltaD = Frame::kDeltaD;
const float KeyFrame::kTgViewZAngleMin = Frame::kTgViewZAngleMin;
const float KeyFrame::kDeltaZForCheckingViewZAngleMin = Frame::kDeltaZForCheckingViewZAngleMin; // [m]

float KeyFrame::skFovCenterDistance = 1.5; // [m] default value, the actual value is set in PointCloudMapping as 0.5*(minPerceptionRange+maxPerceptionRange)

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), 
    mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnLineDGridCols(LINE_D_GRID_COLS), mnLineThetaGridRows(LINE_THETA_GRID_ROWS),
    mfLineGridElementThetaInv(F.mfLineGridElementThetaInv), mfLineGridElementDInv(F.mfLineGridElementDInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy), mDistCoef(F.mDistCoef),
    mbf(F.mbf), mbfInv(F.mbfInv), mb(F.mb), mThDepth(F.mThDepth), 
    // points
    N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), 
    mDescriptors(F.mDescriptors.clone()),
    // lines
    Nlines(F.Nlines), mvKeyLines(F.mvKeyLines), mvKeyLinesUn(F.mvKeyLinesUn),
    mvuRightLineStart(F.mvuRightLineStart), mvDepthLineStart(F.mvDepthLineStart), 
    mvuRightLineEnd(F.mvuRightLineEnd), mvDepthLineEnd(F.mvDepthLineEnd), 
    mLineDescriptors(F.mLineDescriptors.clone()),
    // other data 
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), 
    mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), 
    mvLevelSigma2(F.mvLevelSigma2), mvInvLevelSigma2(F.mvInvLevelSigma2), 
    mnLineScaleLevels(F.mnLineScaleLevels), mfLineScaleFactor(F.mfLineScaleFactor),
    mfLineLogScaleFactor(F.mfLineLogScaleFactor), mvLineScaleFactors(F.mvLineScaleFactors),         
    mvLineLevelSigma2(F.mvLineLevelSigma2), mvLineInvLevelSigma2(F.mvLineInvLevelSigma2),
    mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), mnMaxDiag(F.mnMaxDiag),
    mK(F.mK), 
    mvpMapPoints(F.mvpMapPoints), 
    mvpMapLines(F.mvpMapLines),
    mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), 
    mMedianDepth(F.mMedianDepth),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap),
    mbVisited(false), mbFixed(false)        
{
    mnId=nNextId++;
    mbFixed = (mnId==0); // this check is used in the Optimizer 

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }
    
    if(F.mpLineExtractorLeft)
    {
        mLineGrid.resize(mnLineDGridCols);
        for(int i=0; i<mnLineDGridCols;i++)
        {
            mLineGrid[i].resize(mnLineThetaGridRows);
            for(int j=0; j<mnLineThetaGridRows; j++)
                mLineGrid[i][j] = F.mLineGrid[i][j];
        }
    }

    SetPose(F.mTcw);    
}

KeyFrame::~KeyFrame()
{
#if DEBUG_DESTRUCTOR    
    std::cout << "~KeyFrame(): destroyed id " << mnId << std::endl;
#endif     
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
    
    //fovCw = Ow + Twc.rowRange(0,3).col(2) * skFovCenterDistance;
    fovCw = Ow + Twc.rowRange(0,3).col(2) * mMedianDepth;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

cv::Mat KeyFrame::GetFovCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return fovCw.clone();    
}

void KeyFrame::AddConnection(const KeyFramePtr& pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
#if !ENABLE_NEW_CHANGES            
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
#else
        std::map<KeyFramePtr,int>::iterator it = mConnectedKeyFrameWeights.find(pKF);
        if( it == mConnectedKeyFrameWeights.end() )
        {
            mConnectedKeyFrameWeights[pKF]=weight;             
        }
        else 
        {
            int& kfWeight = it->second; // avoid double map lookup
            kfWeight=weight;
        }
#endif        
    }

    UpdateBestCovisibles();
}


bool sortGreater(const pair<int,KeyFramePtr> &a, const pair<int,KeyFramePtr> &b)
{
    return (a.first > b.first);
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFramePtr> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFramePtr,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));
    
#if !ENABLE_NEW_CHANGES  
    
    sort(vPairs.begin(),vPairs.end());
    list<KeyFramePtr> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFramePtr>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());  
    /*for(size_t i=0, iend=mvOrderedWeights.size(); i<iend;i++)
    { 
        std::cout << "mvOrderedWeights["<<i<<"]: " << mvOrderedWeights[i]  << std::endl;
    }*/    
    
#else    
    
    sort(vPairs.begin(),vPairs.end(), sortGreater ); 
    const size_t sizePairs = vPairs.size();
    mvpOrderedConnectedKeyFrames.resize( sizePairs );
    mvOrderedWeights.resize( sizePairs );
    for(size_t i=0, iend=sizePairs; i<iend;i++)
    {
        //std::cout << "mvOrderedWeights["<<i<<"]: " << mvOrderedWeights[i]  << std::endl;
        mvpOrderedConnectedKeyFrames[i] = vPairs[i].second;
        mvOrderedWeights[i] = vPairs[i].first;
    } 
    
#endif    
    
}

set<KeyFramePtr> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFramePtr> s;
    for(map<KeyFramePtr,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFramePtr> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFramePtr> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFramePtr>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFramePtr> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFramePtr>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFramePtr>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFramePtr>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
#if !ENABLE_NEW_CHANGES       
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
#else
    std::map<KeyFramePtr,int>::iterator it = mConnectedKeyFrameWeights.find(pKF);
    if(it != mConnectedKeyFrameWeights.end())
        return it->second;
    else
        return 0;    
#endif    
}

void KeyFrame::AddMapPoint(const MapPointPtr& pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPointPtr>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPointPtr& pMP)
{
    int idx = pMP->GetIndexInKeyFrame(WrapPtr(this));
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPointPtr>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPointPtr pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPointPtr> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPointPtr> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPointPtr pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPointPtr pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPointPtr> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPointPtr KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

// Lines

void KeyFrame::AddMapLine(const MapLinePtr& pML, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    mvpMapLines[idx]=pML;
}

void KeyFrame::EraseMapLineMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    mvpMapLines[idx]=static_cast<MapLinePtr>(NULL);
}

void KeyFrame::EraseMapLineMatch(MapLinePtr& pML)
{
    int idx = pML->GetIndexInKeyFrame(WrapPtr(this));
    if(idx>=0)
        mvpMapLines[idx]=static_cast<MapLinePtr>(NULL);
}

void KeyFrame::ReplaceMapLineMatch(const size_t &idx, MapLinePtr pML)
{
    mvpMapLines[idx]=pML;
}

set<MapLinePtr> KeyFrame::GetMapLines()
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    set<MapLinePtr> s;
    for(size_t i=0, iend=mvpMapLines.size(); i<iend; i++)
    {
        if(!mvpMapLines[i])
            continue;
        MapLinePtr& pML = mvpMapLines[i];
        if(!pML->isBad())
            s.insert(pML);
    }
    return s;
}

vector<MapLinePtr> KeyFrame::GetMapLineMatches()
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    return mvpMapLines;
}


MapLinePtr KeyFrame::GetMapLine(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    return mvpMapLines[idx];
}

int KeyFrame::TrackedMapLines(const int &minObs)
{
    unique_lock<mutex> lock(mMutexLineFeatures);

    int nLines=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<Nlines; i++)
    {
        MapLinePtr& pML = mvpMapLines[i];
        if(pML)
        {
            if(!pML->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapLines[i]->Observations()>=minObs)
                        nLines++;
                }
                else
                    nLines++;
            }
        }
    }

    return nLines;
}

// Objects 

void KeyFrame::AddMapObject(const MapObjectPtr& pMObj)
{
    unique_lock<mutex> lock(mMutexObjects);
    mvpMapObjects.push_back(pMObj);
}

void KeyFrame::EraseMapObjectMatch(const size_t &id)
{
    unique_lock<mutex> lock(mMutexObjects);     
    std::vector<MapObjectPtr >::iterator it=mvpMapObjects.begin(), itEnd=mvpMapObjects.end();    
    while(it != itEnd)
    {
        if((*it)->mnId == id)
        {
            it = mvpMapObjects.erase(it);       
        }
        else
            it++;
    }       
}

void KeyFrame::EraseMapObjectMatch(MapObjectPtr& pMObj)
{
    unique_lock<mutex> lock(mMutexObjects);    
    //https://thispointer.com/removing-all-occurences-of-an-element-from-vector-in-on-complexity/
    mvpMapObjects.erase(std::remove(mvpMapObjects.begin(), mvpMapObjects.end(), pMObj), mvpMapObjects.end());
}

void KeyFrame::ReplaceMapObjectMatch(MapObjectPtr pMObjToReplace, MapObjectPtr pMObj)
{
    unique_lock<mutex> lock(mMutexObjects);    
    std::vector<MapObjectPtr >::iterator it=mvpMapObjects.begin(), itEnd=mvpMapObjects.end();   
    while(it != itEnd)
    {
        if((*it)->mnId == pMObjToReplace->mnId)
        {
            mvpMapObjects.erase(it);
            break;
        }
    }    
    mvpMapObjects.push_back(pMObj);
}

set<MapObjectPtr > KeyFrame::GetMapObjects()
{
    unique_lock<mutex> lock(mMutexObjects);
    set<MapObjectPtr > s;
    for(size_t i=0, iend=mvpMapObjects.size(); i<iend; i++)
    {
        MapObjectPtr pMObj = mvpMapObjects[i];
        if(!pMObj->isBad())
            s.insert(pMObj);
    }
    return s;
}

vector<MapObjectPtr > KeyFrame::GetMapObjectMatches()
{
    unique_lock<mutex> lock(mMutexObjects);
    return mvpMapObjects;
}


//MapObjectPtr KeyFrame::GetMapObject(const size_t &id)
//{
//    unique_lock<mutex> lock(mMutexObjects);
//
//    std::vector<MapObjectPtr >::iterator it = mvpMapObjects.begin();
//    while(it != mvpMapObjects.end())
//    {
//        if( (*it)->mnId == id)
//        {
//            return *it;
//        }
//    }      
//    
//    return MapObjectPtr(); // TO BE FIXED: here we must return something or the method signature should be completely changed
//}

int KeyFrame::TrackedMapObjects(const int &minObs)
{
    unique_lock<mutex> lock(mMutexObjects);

    int nObjects=0;
    const bool bCheckObs = minObs>0;
    
    std::vector<MapObjectPtr >::iterator it = mvpMapObjects.begin();
    while(it != mvpMapObjects.end())
    {
        if(!((*it)->isBad()))        
        {
            if(bCheckObs)
            {
                if((*it)->GetNumObservations()>=minObs)
                    nObjects++;
            }
            else
                nObjects++;
        }
    }      

    return nObjects;
}

//

void KeyFrame::UpdateConnections()
{
    map<KeyFramePtr,int> KFcounter;

    vector<MapPointPtr> vpMPoints;
    vector<MapLinePtr> vpMLines;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMPoints = mvpMapPoints;
    }
    
    {
        unique_lock<mutex> lockMLs(mMutexLineFeatures);
        vpMLines = mvpMapLines;
    }    

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPointPtr>::iterator vit=vpMPoints.begin(), vend=vpMPoints.end(); vit!=vend; vit++)
    {
        MapPointPtr pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFramePtr,size_t> observations = pMP->GetObservations();

        for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }
    
    //For all map lines in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    const int lineWeight = round( Tracking::sknLineTrackWeigth );
    for(vector<MapLinePtr>::iterator vit=vpMLines.begin(), vend=vpMLines.end(); vit!=vend; vit++)
    {
        MapLinePtr& pML = *vit;

        if(!pML)
            continue;

        if(pML->isBad())
            continue;

        map<KeyFramePtr,size_t> observations = pML->GetObservations();

        for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first] += lineWeight;
        }
    }    
    
    // TODO: add objects ?

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFramePtr pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFramePtr> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFramePtr,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(WrapPtr(this),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(WrapPtr(this),nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFramePtr> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFramePtr>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        // Update spanning tree (? this does guarantee no loops ?)
        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(WrapPtr(this));
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(const KeyFramePtr& pKF)
{
#if ENABLE_CHANGES_SELFCHILD_ISSUE    
    if( GetRawPtr(pKF) == this) return;     
#endif
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFramePtr& pKF)
{
#if ENABLE_CHANGES_SELFCHILD_ISSUE       
    if( GetRawPtr(pKF) == this) return; 
#endif
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(WrapPtr(this)); // N.B.: we use a wrap pointer (empty deleter) since the raw ptr 'this' has not been created with the wrap pointer 
}

set<KeyFramePtr> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFramePtr KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFramePtr& pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFramePtr> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFramePtr,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(WrapPtr(this));

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(WrapPtr(this));
    
    for(size_t i=0; i<mvpMapLines.size(); i++)
        if(mvpMapLines[i])
            mvpMapLines[i]->EraseObservation(WrapPtr(this));
    
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexLineFeatures);        

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFramePtr> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFramePtr pC = 0;
            KeyFramePtr pP = 0;

            for(set<KeyFramePtr>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFramePtr pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the child keyframe
                vector<KeyFramePtr> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFramePtr>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue) 
            {
#if ENABLE_CHANGES_SELFCHILD_ISSUE   
                if(pC == pP) { std::cout << "KeyFrame::SetBadFlag() - pC == pP" << std::endl; }
                //if(pC != pP)
#endif                    
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFramePtr>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(WrapPtr(this));
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(WrapPtr(this));
    mpKeyFrameDB->erase(WrapPtr(this));
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(const KeyFramePtr& pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
#if !ENABLE_NEW_CHANGES         
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
#else
        std::map<KeyFramePtr,int>::iterator it = mConnectedKeyFrameWeights.find(pKF);
        if(it != mConnectedKeyFrameWeights.end())
        {
            mConnectedKeyFrameWeights.erase(it);
            bUpdate=true;
        }        
#endif        
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t>& vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


vector<size_t> KeyFrame::GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, const float& dtheta, const float& dd) const
{    
    Line2DRepresentation lineRepresentation;
    Geom2DUtils::GetLine2dRepresentation(xs, ys, xe, ye, lineRepresentation);
    return GetLineFeaturesInArea(lineRepresentation,dtheta,dd);
}

//vector<size_t> KeyFrame::GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, const float& dtheta, const float& dd) const
//{       
//    vector<size_t> vIndices;
//    vIndices.reserve(Nlines);
//  
//    const int nMinCellThetaRow = (int)floor((lineRepresentation.theta - dtheta -LINE_THETA_MIN)*mfLineGridElementThetaInv);
//    const int nMaxCellThetaRow = (int)floor((lineRepresentation.theta + dtheta -LINE_THETA_MIN)*mfLineGridElementThetaInv);
// 
//    
//    const int nMinCellDCol = std::max(0,(int)floor((lineRepresentation.d - dd + mnMaxDiag)*mfLineGridElementDInv));
//    if( nMinCellDCol >= LINE_D_GRID_COLS)
//    {
//        //std::cout << "error in getting line feature d: " << lineRepresentation.d << ", nMinCellDCol: " << nMinCellDCol << std::endl; 
//        return vIndices; 
//    }
//    
//    const int nMaxCellDCol = std::min(LINE_D_GRID_COLS-1,(int)floor((lineRepresentation.d + dd + mnMaxDiag)*mfLineGridElementDInv));
//    if( nMaxCellDCol < 0)
//    {
//        //std::cout << "error in getting line feature d: " << lineRepresentation.d << ", nMaxCellDCol: " << nMaxCellDCol << std::endl; 
//        return vIndices; 
//    }
//    
//    //std::cout << "returning cell [" <<nMinCellDCol<<", " <<  nMaxCellDCol<< "]" << std::endl; 
//
//    for(int ix = nMinCellDCol; ix<=nMaxCellDCol; ix++)
//    {
//        for(int iy = nMinCellThetaRow; iy<=nMaxCellThetaRow; iy++)
//        {
//            int iyW = Utils::Modulus(iy,LINE_THETA_GRID_ROWS);
//            const vector<size_t>& vCell = mLineGrid[ix][iyW];
//            if(vCell.empty())
//                continue;
//
//            vIndices.insert(vIndices.end(),vCell.begin(),vCell.end());
////            for(size_t j=0, jend=vCell.size(); j<jend; j++)
////            {
////                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
////
////
////                const float distx = kpUn.pt.x-x;
////                const float disty = kpUn.pt.y-y;
////
////                if(fabs(distx)<r && fabs(disty)<r)
////                    vIndices.push_back(vCell[j]);
////            }
//        }
//    }
//    return vIndices;
//}


vector<size_t> KeyFrame::GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, const float& dtheta, const float& dd) const
{       
    vector<size_t> vIndices;
    vIndices.reserve(Nlines);
            
    const float thetaMin = lineRepresentation.theta - dtheta;
    const float thetaMax = lineRepresentation.theta + dtheta;
        
    if( fabs(thetaMin - thetaMax) > M_PI )
    {
        std::cout << "KeyFrame::GetLineFeaturesInArea() - ERROR - you are searching over the full theta interval!" << std::endl; 
        quick_exit(-1);
        return vIndices; 
    }      
    
    const float dMin = lineRepresentation.d - dd;
    const float dMax = lineRepresentation.d + dd;    
    
    GetLineFeaturesInArea(thetaMin, thetaMax, dMin, dMax, vIndices);

    return vIndices;
}

void KeyFrame::GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, vector<size_t>& vIndices) const
{                    
    if( thetaMin < -M_PI_2 )
    {
        // let's split the search interval in two intervals (we are searching over a manifold here)
        // 1) bring theta angles within [-pi/2, pi/2]
        // 2) if you wrap around +-pi/2 then you have to invert the sign of d

        const float thetaMin1 = thetaMin + M_PI; 
        const float thetaMax1 = M_PI_2 - std::numeric_limits<float>::epsilon();
        const float dMin1 = -dMax; 
        const float dMax1 = -dMin; 
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,vIndices);
        
        const float thetaMin2 = -M_PI_2 + std::numeric_limits<float>::epsilon(); 
        const float thetaMax2 = thetaMax;
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,vIndices);
        
        return; // < EXIT 
    }
    
    if( thetaMax > M_PI_2 )
    {
        // let's split the search interval in two intervals (we are searching over a manifold here)
        // 1) bring theta angles within [-pi/2, pi/2]
        // 2) if you wrap around +-pi/2 then you have to invert the sign of d 
        
        const float thetaMin1 = -M_PI_2 + std::numeric_limits<float>::epsilon(); 
        const float thetaMax1 = thetaMax - M_PI;
        const float dMin1 = -dMax; 
        const float dMax1 = -dMin; 
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,vIndices);
        
        const float thetaMin2 = thetaMin; 
        const float thetaMax2 = M_PI_2 - std::numeric_limits<float>::epsilon();
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,vIndices);
        
        return; // < EXIT         
        
    }    
    
    //const int nMinCellThetaRow = (int)floor((thetaMin -LINE_THETA_MIN)*mfLineGridElementThetaInv);
    //const int nMaxCellThetaRow = (int)floor((thetaMax -LINE_THETA_MIN)*mfLineGridElementThetaInv);
    
    const int nMinCellThetaRow = std::max(0,(int)floor((thetaMin -LINE_THETA_MIN)*mfLineGridElementThetaInv));
    if( nMinCellThetaRow >= LINE_THETA_GRID_ROWS)
    {
        return; 
    }    
    const int nMaxCellThetaRow = std::min(LINE_THETA_GRID_ROWS-1, (int)floor((thetaMax -LINE_THETA_MIN)*mfLineGridElementThetaInv));    
    if( nMaxCellThetaRow < 0)
    {
        return; 
    }
    
    const int nMinCellDCol = std::max(0,(int)floor((dMin + mnMaxDiag)*mfLineGridElementDInv));  // + mnMaxDiag = - mnMinDiag
    if( nMinCellDCol >= LINE_D_GRID_COLS)
    {
        return; 
    }
    const int nMaxCellDCol = std::min(LINE_D_GRID_COLS-1,(int)floor((dMax + mnMaxDiag)*mfLineGridElementDInv)); // + mnMaxDiag = - mnMinDiag
    if( nMaxCellDCol < 0)
    {
        return; 
    }
        
    for(int ix = nMinCellDCol; ix<=nMaxCellDCol; ix++)
    {
        for(int iy = nMinCellThetaRow; iy<=nMaxCellThetaRow; iy++)
        {
            //const int iyW = Utils::Modulus(iy,LINE_THETA_GRID_ROWS); 
            const int iyW = iy;
            const vector<size_t>& vCell = mLineGrid[ix][iyW];
            if(vCell.empty())
                continue;

            vIndices.insert(vIndices.end(),vCell.begin(),vCell.end()); 
        }
    }
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(const int& i)
{
    const float& z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}


bool KeyFrame::UnprojectStereoLine(const int& i, cv::Mat& p3DStart, cv::Mat& p3DEnd)
{
    bool res = true; 
    
    const float& zS = mvDepthLineStart[i];
    const float& zE = mvDepthLineEnd[i];
    if( (zS>0) && (zE>0) )
    {
        const float uS = mvKeyLinesUn[i].startPointX;
        const float vS = mvKeyLinesUn[i].startPointY;
        const float xS = (uS-cx)*zS*invfx;
        const float yS = (vS-cy)*zS*invfy;
        const cv::Mat xS3Dc = (cv::Mat_<float>(3,1) << xS, yS, zS);
        
        const float uE = mvKeyLinesUn[i].endPointX;
        const float vE = mvKeyLinesUn[i].endPointY;
        const float xE = (uE-cx)*zE*invfx;
        const float yE = (vE-cy)*zE*invfy;
        const cv::Mat xE3Dc = (cv::Mat_<float>(3,1) << xE, yE, zE);
        
        unique_lock<mutex> lock(mMutexPose);
        const cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        const cv::Mat Ow  = Twc.rowRange(0,3).col(3);
        p3DStart = Rwc*xS3Dc+Ow;
        p3DEnd   = Rwc*xE3Dc+Ow;
        
        res = true; 
        
        /*const float deltaDepth = fabs(zS-zE);
        if(deltaDepth > kDeltaZForCheckingViewZAngleMin)
        {
            //const cv::Mat line3Dc = xS3Dc-xE3Dc;
            //const float tgViewZAngle = sqrt( Utils::Pow2(line3Dc.at<float>(0)) + Utils::Pow2(line3Dc.at<float>(1)))/fabs(line3Dc.at<float>(3));
            const float tgViewZAngle = sqrt( Utils::Pow2(xS-xE) + Utils::Pow2(yS-yE) )/deltaDepth;
            if(tgViewZAngle<kTgViewZAngleMin)
                res = false;
        }*/
    }
    else
    {
        p3DStart = cv::Mat();
        p3DEnd   = cv::Mat();
        std::cout << "********************************************************************" << std::endl;        
        std::cout << "KeyFrame::UnprojectStereoLine() - WARNING unprojecting invalid line!" << std::endl;
        std::cout << "********************************************************************" << std::endl;
        res = false; 
    }
    
    return res; 
    
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{    
    vector<MapPointPtr> vpMapPoints; 
    vector<MapLinePtr> vpMapLines;     
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock3(mMutexLineFeatures);        
        vpMapPoints = mvpMapPoints;     
        vpMapLines = mvpMapLines;             
    
        unique_lock<mutex> lock2(mMutexPose);        
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    
    for(int i=0; i<N; i++)
    {
        //if(mvpMapPoints[i])
        if(vpMapPoints[i])
        {
            //MapPointPtr& pMP = mvpMapPoints[i];            
            MapPointPtr& pMP = vpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }
    
    for(int i=0; i<Nlines; i++)
    {
        //if(mvpMapLines[i])        
        if(vpMapLines[i])
        {
            //MapLinePtr& pML = mvpMapLines[i];
            MapLinePtr& pML = vpMapLines[i];
            //cv::Mat x3DSw = pML->GetWorldPosStart();
            //cv::Mat x3DEw = pML->GetWorldPosEnd();
            cv::Mat x3DSw, x3DEw;
            pML->GetWorldEndPoints(x3DSw, x3DEw);            
            cv::Mat x3DMw = 0.5*(x3DSw + x3DEw);
            float z = Rcw2.dot(x3DMw)+zcw;
            vDepths.push_back(z);
        }
    }    

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}


// Default serializing Constructor
KeyFrame::KeyFrame(): 
    mnFrameId(0),  mTimeStamp(0), 
    mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(0), mfGridElementHeightInv(0),
    mnLineDGridCols(LINE_D_GRID_COLS), mnLineThetaGridRows(LINE_THETA_GRID_ROWS),
    mfLineGridElementThetaInv(0), mfLineGridElementDInv(0),        
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), //mDistCoef(0),
    mbf(0), mbfInv(0), mb(0), mThDepth(0), 
    // points
    N(0), //mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    //mvuRight(F.mvuRight), mvDepth(F.mvDepth), 
    //mDescriptors(F.mDescriptors.clone()),
    // lines
    Nlines(0), //mvKeyLines(F.mvKeyLines), mvKeyLinesUn(F.mvKeyLinesUn),
    //mvuRightLineStart(F.mvuRightLineStart), mvDepthLineStart(F.mvDepthLineStart), 
    //mvuRightLineEnd(F.mvuRightLineEnd), mvDepthLineEnd(F.mvDepthLineEnd), 
    //mLineDescriptors(F.mLineDescriptors.clone()),        
    // other data 
    //mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), 
    mnScaleLevels(0), mfScaleFactor(0),
    mfLogScaleFactor(0), //mvScaleFactors(F.mvScaleFactors), 
    //mvLevelSigma2(F.mvLevelSigma2), mvInvLevelSigma2(F.mvInvLevelSigma2), 
    mnLineScaleLevels(0), mfLineScaleFactor(0),
    mfLineLogScaleFactor(0), //mvLineScaleFactors(F.mvLineScaleFactors),         
    //mvLineLevelSigma2(F.mvLineLevelSigma2), mvLineInvLevelSigma2(F.mvLineInvLevelSigma2),         
    mnMinX(0), mnMinY(0), mnMaxX(0), mnMaxY(0), mnMaxDiag(0),
    //mK(F.mK), 
    //mvpMapPoints(F.mvpMapPoints), 
    //mvpMapLines(F.mvpMapLines),
    mpKeyFrameDB(0),
    mpORBvocabulary(0), 
    mMedianDepth(KeyFrame::skFovCenterDistance),    
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(0), mpMap(0),
    mbVisited(false), mbFixed(false)
{}

template < class Archive >
void KeyFrame::serialize(Archive &ar, const unsigned int version) 
{
    UNUSED_VAR(version);
        
    // no mutex needed vars
    ar &nNextId;
    ar &mnId; 
    mbFixed = (mnId==0); // this check is used in the Optimizer 
    
    ar &const_cast< long unsigned int & >(mnFrameId);
    ar &const_cast< double & >(mTimeStamp);
    
    // Grid related vars
    ar &const_cast< int & >(mnGridCols);
    ar &const_cast< int & >(mnGridRows);
    ar &const_cast< float & >(mfGridElementWidthInv);
    ar &const_cast< float & >(mfGridElementHeightInv);
    
    ar &const_cast< int & >(mnLineDGridCols);
    ar &const_cast< int & >(mnLineThetaGridRows);
    ar &const_cast< float & >(mfLineGridElementThetaInv);
    ar &const_cast< float & >(mfLineGridElementDInv);
        
    // Tracking related vars
    ar &mnTrackReferenceForFrame &mnFuseTargetForKF;
    // LocalMaping related vars
    ar &mnBALocalForKF &mnBAFixedForKF;
    // KeyFrameDB related vars
    ar &mnLoopQuery &mnLoopWords &mLoopScore &mnRelocQuery &mnRelocWords
        &mRelocScore;
    // LoopClosing related vars
    ar &mTcwGBA &mTcwBefGBA &mnBAGlobalForKF;
    // calibration parameters
    ar &const_cast< float & >(fx) & const_cast< float & >(fy) &
        const_cast< float & >(cx) & const_cast< float & >(cy);
    ar &const_cast< float & >(invfx) & const_cast< float & >(invfy) &
        const_cast< float & >(mbf) & const_cast< float & >(mbfInv);
    ar &const_cast< float & >(mb) & const_cast< float & >(mThDepth);
    
    // Number of KeyPoints;
    ar &const_cast< int & >(N);
    // KeyPoints, stereo coordinate and descriptors
    ar &const_cast< std::vector< cv::KeyPoint > & >(mvKeys);
    ar &const_cast< std::vector< cv::KeyPoint > & >(mvKeysUn);
    ar &const_cast< std::vector< float > & >(mvuRight);
    ar &const_cast< std::vector< float > & >(mvDepth);
    ar &const_cast< cv::Mat & >(mDescriptors);
    
    // Number of KeyLines;
    ar &const_cast< int & >(Nlines);
    // KeyLines, stereo coordinate and descriptors (all associated by an index)
    ar &const_cast< std::vector< cv::line_descriptor_c::KeyLine> & >(mvKeyLines);
    ar &const_cast< std::vector< cv::line_descriptor_c::KeyLine > & >(mvKeyLinesUn);
    ar &const_cast< std::vector< float > & >(mvuRightLineStart);
    ar &const_cast< std::vector< float > & >(mvDepthLineStart);
    ar &const_cast< std::vector< float > & >(mvuRightLineEnd);
    ar &const_cast< std::vector< float > & >(mvDepthLineEnd);
    ar &const_cast< cv::Mat & >(mLineDescriptors);    
        
    // Bow
    ar &mBowVec &mFeatVec;
    // Pose relative to parent
    ar &mTcp;
    
    // Scale related
    ar &const_cast< int & >(mnScaleLevels) &
        const_cast< float & >(mfScaleFactor) &
        const_cast< float & >(mfLogScaleFactor);
    ar &const_cast< std::vector< float > & >(mvScaleFactors) &
        const_cast< std::vector< float > & >(mvLevelSigma2) &
        const_cast< std::vector< float > & >(mvInvLevelSigma2);
    
    ar &const_cast< int & >(mnLineScaleLevels) &
        const_cast< float & >(mfLineScaleFactor) &
        const_cast< float & >(mfLineLogScaleFactor);
    ar &const_cast< std::vector< float > & >(mvLineScaleFactors) &
        const_cast< std::vector< float > & >(mvLineLevelSigma2) &
        const_cast< std::vector< float > & >(mvLineInvLevelSigma2);
    
    // Image bounds and calibration
    ar &const_cast< int & >(mnMinX) & const_cast< int & >(mnMinY) &
        const_cast< int & >(mnMaxX) & const_cast< int & >(mnMaxY) & const_cast< int & >(mnMaxDiag);
    ar &const_cast< cv::Mat & >(mK);

    // mutex needed vars, but don't lock mutex in the save/load procedure
    {
        unique_lock<mutex> lock_pose(mMutexPose);
        ar &Tcw &Twc &Ow &Cw;
    }
    {
        unique_lock<mutex> lock_feature(mMutexFeatures);
        ar &mvpMapPoints;  // hope boost deal with the pointer graph well
    }
    {
        unique_lock<mutex> lock_feature(mMutexLineFeatures);
        ar &mvpMapLines;
    }    
    // BoW
    ar &mpKeyFrameDB;
    // mpORBvocabulary restore elsewhere(see SetORBvocab)
    {
        // Grid related
        unique_lock< mutex > lock_connection(mMutexConnections);
        ar &mGrid &mLineGrid &mConnectedKeyFrameWeights &mvpOrderedConnectedKeyFrames
            &mvOrderedWeights;
        // Spanning Tree and Loop Edges
        ar &mbFirstConnection &mpParent &mspChildrens &mspLoopEdges;
        // Bad flags
        ar &mbNotErase &mbToBeErased &mbBad &mHalfBaseline;
    }
    // Map of Points and Lines
    ar &mpMap;
    // don't save mutex
    
    // allocate line grid in case the loaded map did not manage lines 
    if( mLineGrid.empty() )
    {
        mLineGrid.resize(mnLineDGridCols);
        for(int i=0; i<mnLineDGridCols;i++)
        {
            mLineGrid[i].resize(mnLineThetaGridRows);
        }    
    }
    
}
template void KeyFrame::serialize(boost::archive::binary_iarchive &, const unsigned int);
template void KeyFrame::serialize(boost::archive::binary_oarchive &, const unsigned int);

} //namespace PLVS
