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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "ImuTypes.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapObject.h"
#include "Tracking.h"
#include "Geom2DUtils.h"
#include "Utils.h"

#include<mutex>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#define ENABLE_CHANGES_SELFCHILD_ISSUE 1
#define ENABLE_NEW_CHANGES 1
#define DEBUG_DESTRUCTOR 1
  

namespace PLVS2
{

long unsigned int KeyFrame::nNextId=0;

const std::vector<size_t> KeyFrame::kEmptyVecSizet = std::vector<size_t>();

const float KeyFrame::kDeltaTheta = Frame::kDeltaTheta;
const float KeyFrame::kDeltaD = Frame::kDeltaD;
const float KeyFrame::kTgViewZAngleMin = Frame::kTgViewZAngleMin;
const float KeyFrame::kDeltaZForCheckingViewZAngleMin = Frame::kDeltaZForCheckingViewZAngleMin; // [m]

float KeyFrame::skFovCenterDistance = 1.5; // [m] default value, the actual value is set in PointCloudMapping as 0.5*(minPerceptionRange+maxPerceptionRange)

KeyFrame::KeyFrame():
        mnFrameId(0),  mTimeStamp(0), 
        mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(0), mfGridElementHeightInv(0),
        mnLineDGridCols(LINE_D_GRID_COLS), mnLineThetaGridRows(LINE_THETA_GRID_ROWS),
        mfLineGridElementThetaInv(0), mfLineGridElementDInv(0),        
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnMergeQuery(0), mnMergeWords(0), mnBAGlobalForKF(0),
        fx(0), fy(0), cx(0), cy(0), invfx(0), invfy(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
        mbf(0), mbfInv(0), mb(0), mThDepth(0), 
        // points
        N(0), mvKeys(static_cast<vector<cv::KeyPoint> >(NULL)), mvKeysUn(static_cast<vector<cv::KeyPoint> >(NULL)),
        mvuRight(static_cast<vector<float> >(NULL)), mvDepth(static_cast<vector<float> >(NULL)), /*mDescriptors(NULL),*/
        /*mBowVec(NULL), mFeatVec(NULL),*/ 
        mnScaleLevels(0), mfScaleFactor(0),
        mfLogScaleFactor(0), mvScaleFactors(0), mvLevelSigma2(0),
        mvInvLevelSigma2(0),
        // lines
        Nlines(0), //mvKeyLines(F.mvKeyLines), mvKeyLinesUn(F.mvKeyLinesUn),
        //mvuRightLineStart(F.mvuRightLineStart), mvDepthLineStart(F.mvDepthLineStart), 
        //mvuRightLineEnd(F.mvuRightLineEnd), mvDepthLineEnd(F.mvDepthLineEnd), 
        //mLineDescriptors(F.mLineDescriptors.clone()),   
        mnLineScaleLevels(0), mfLineScaleFactor(0),
        mfLineLogScaleFactor(0), mvLineScaleFactors(0), mvLineLevelSigma2(0), 
        mvLineInvLevelSigma2(0),  
        // other data 
        mnMinX(0), mnMinY(0), mnMaxX(0), mnMaxY(0), mnMaxDiag(0), 
        mpKeyFrameDB(0),
        mpORBvocabulary(0), 
        mMedianDepth(KeyFrame::skFovCenterDistance),
        /*mK(NULL),*/  
        mPrevKF(static_cast<KeyFramePtr>(NULL)), mNextKF(static_cast<KeyFramePtr>(NULL)), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
        mbToBeErased(false), mbBad(false), mHalfBaseline(0), mbCurrentPlaceRecognition(false), mnMergeCorrectedForKF(0), 
        NLeft(-1),NRight(-1), mnNumberOfOpt(0),
        NlinesLeft(-1),NlinesRight(-1), 
        mpMap(0),        
        mbVisited(false), mbFixed(false),
        mbHasVelocity(false)
{

}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    bImu(pMap->isImuInitialized()), mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), 
    mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnLineDGridCols(LINE_D_GRID_COLS), mnLineThetaGridRows(LINE_THETA_GRID_ROWS),
    mfLineGridElementThetaInv(F.mfLineGridElementThetaInv), mfLineGridElementDInv(F.mfLineGridElementDInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0), mnBALocalForMerge(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0), mnPlaceRecognitionQuery(0), mnPlaceRecognitionWords(0), mPlaceRecognitionScore(0),
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
    mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX), mnMaxY(F.mnMaxY), 
    mnMaxDiag(F.mnMaxDiag),
    mK(F.mK.clone()), 
    mK_(F.mK_), 
    mPrevKF(NULL), mNextKF(NULL), 
    mpImuPreintegrated(F.mpImuPreintegrated), mImuCalib(F.mImuCalib), 
    mvpMapPoints(F.mvpMapPoints), 
    mvpMapLines(F.mvpMapLines),
    mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), 
    mMedianDepth(F.mMedianDepth),
    mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mnDataset(F.mnDataset),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap), mbCurrentPlaceRecognition(false), mNameFile(F.mNameFile), mnMergeCorrectedForKF(0),
    mpCamera(F.mpCamera), mpCamera2(F.mpCamera2),
    mvLeftToRightMatch(F.mvLeftToRightMatch),mvRightToLeftMatch(F.mvRightToLeftMatch), mTlr(F.GetRelativePoseTlr()),
    mvKeysRight(F.mvKeysRight), NLeft(F.Nleft), NRight(F.Nright), 
    mvKeyLinesRight(F.mvKeyLinesRight), mvKeyLinesRightUn(F.mvKeyLinesRightUn), NlinesLeft(F.NlinesLeft), NlinesRight(F.NlinesRight), 
    mvLeftToRightLinesMatch(F.mvLeftToRightLinesMatch),mvRightToLeftLinesMatch(F.mvRightToLeftLinesMatch),
    mTrl(F.GetRelativePoseTrl()), mnNumberOfOpt(0),
    mbVisited(false), mbFixed(false),   
    mbHasVelocity(false)
{

    imgLeft = F.imgLeft.clone();
    imgRight = F.imgRight.clone();

    mnId=nNextId++;
    mbFixed = (mnId==pMap->GetInitKFid()); // this check is used in the Optimizer 

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }

    if(F.mpLineExtractorLeft)
    {
        mLineGrid.resize(mnLineDGridCols);
        if(F.NlinesLeft != -1)  mLineGridRight.resize(mnLineDGridCols);
        for(int i=0; i<mnLineDGridCols;i++)
        {
            mLineGrid[i].resize(mnLineThetaGridRows);
            if(F.NlinesLeft != -1) mLineGridRight[i].resize(mnLineThetaGridRows);
            for(int j=0; j<mnLineThetaGridRows; j++){
                mLineGrid[i][j] = F.mLineGrid[i][j];
                if(F.NlinesLeft != -1){
                    mLineGridRight[i][j] = F.mLineGridRight[i][j];
                }
            }
        }
    }


    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    mnOriginMapId = pMap->GetId();
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

void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexPose);

    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    Ow =  mTwc.translation(); //-mRwc*mTcw.translation();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }

    Eigen::Vector3f center;
    center << mHalfBaseline, 0 , 0;
    Cw = mRwc * center + mTwc.translation();

    //fovCw = Ow + Twc.rowRange(0,3).col(2) * skFovCenterDistance;
    fovCw = Ow + mRwc.col(2) * mMedianDepth;
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
{
    unique_lock<mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter(){
    unique_lock<mutex> lock(mMutexPose);
    return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition()
{
    unique_lock<mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation(){
    unique_lock<mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetFovCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return fovCw;    
}

Eigen::Vector3f KeyFrame::GetVelocity()
{
    unique_lock<mutex> lock(mMutexPose);
    return mVw;
}

bool KeyFrame::isVelocitySet()
{
    unique_lock<mutex> lock(mMutexPose);
    return mbHasVelocity;
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
    {
       if(mit->first->isBad()) continue; // Luigi: added this 
       vPairs.push_back(make_pair(mit->second,mit->first));
    }

#if !ENABLE_NEW_CHANGES  
    
    sort(vPairs.begin(),vPairs.end());
    list<KeyFramePtr> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
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
    {
        return vector<KeyFramePtr>();
    }

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return vector<KeyFramePtr>();
    }
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

int KeyFrame::GetNumberMPs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
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
    tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(WrapPtr(this));
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapPoints[leftIndex]=static_cast<MapPointPtr>(NULL);
    if(rightIndex != -1)
        mvpMapPoints[rightIndex]=static_cast<MapPointPtr>(NULL);
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
        const MapPointPtr& pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

unordered_set<MapPointPtr> KeyFrame::GetMapPointsUnordered()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unordered_set<MapPointPtr> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        const MapPointPtr& pMP = mvpMapPoints[i];
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
    tuple<size_t,size_t> indexes = pML->GetIndexInKeyFrame(WrapPtr(this));
    size_t leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    if(leftIndex != -1)
        mvpMapLines[leftIndex]=static_cast<MapLinePtr>(NULL);
    if(rightIndex != -1)
        mvpMapLines[rightIndex]=static_cast<MapLinePtr>(NULL);
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
        const MapLinePtr& pML = mvpMapLines[i];
        if(!pML->isBad())
            s.insert(pML);
    }
    return s;
}

std::unordered_set<MapLinePtr> KeyFrame::GetMapLinesUnordered()
{
    unique_lock<mutex> lock(mMutexLineFeatures);
    std::unordered_set<MapLinePtr> s;
    for(size_t i=0, iend=mvpMapLines.size(); i<iend; i++)
    {
        if(!mvpMapLines[i])
            continue;
        const MapLinePtr& pML = mvpMapLines[i];
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
    set<MapObjectPtr> s;
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

void KeyFrame::UpdateConnections(bool upParent)
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

        map<KeyFramePtr,tuple<int,int>> observations = pMP->GetObservations();

        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
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

        map<KeyFramePtr,tuple<int,int>> observations = pML->GetObservations();

        for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
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
    if(!upParent)
        cout << "UPDATE_CONN: current KF " << mnId << endl;
    for(map<KeyFramePtr,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << endl;
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

        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFramePtr>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        // Update spanning tree (? this does guarantee no loops ?)
        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
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
    if(GetRawPtr(pKF) == this)
    {
        cout << "ERROR: Change parent KF, the parent and child are the same KF" << endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

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

void KeyFrame::SetFirstConnection(bool bFirst)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
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

void KeyFrame::AddMergeEdge(KeyFramePtr pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(pKF);
}

set<KeyFramePtr> KeyFrame::GetMergeEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspMergeEdges;
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
    MSG_ASSERT(mpMap,"mpMap not set");
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFramePtr,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(WrapPtr(this));
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(WrapPtr(this));
        }
    }

    
    for(size_t i=0; i<mvpMapLines.size(); i++)
    {
        if(mvpMapLines[i])
        {
            mvpMapLines[i]->EraseObservation(WrapPtr(this));
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexLineFeatures);        

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFramePtr> sParentCandidates;
        if(mpParent)
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
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(set<KeyFramePtr>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(WrapPtr(this));
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
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


vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const auto& grid = (!bRight) ? mGrid : mGridRight;
    const auto& keys = (NLeft == -1) ? mvKeysUn : 
                                      (!bRight) ? mvKeys : mvKeysRight;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            //const vector<size_t>& vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            const vector<size_t>& vCell = grid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                // const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                //                                          : (!bRight) ? mvKeys[vCell[j]]
                //                                                      : mvKeysRight[vCell[j]];
                const cv::KeyPoint &kpUn = keys[vCell[j]];                                                                     
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}


vector<size_t> KeyFrame::GetLineFeaturesInArea(const float &xs, const float  &ys, const float &xe, const float  &ye, const float& dtheta, const float& dd, const bool bRight) const
{    
    Line2DRepresentation lineRepresentation;
    Geom2DUtils::GetLine2dRepresentation(xs, ys, xe, ye, lineRepresentation);
    return GetLineFeaturesInArea(lineRepresentation,dtheta,dd,bRight);
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


vector<size_t> KeyFrame::GetLineFeaturesInArea(const Line2DRepresentation& lineRepresentation, const float& dtheta, const float& dd, const bool bRight) const
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
    
    GetLineFeaturesInArea(thetaMin, thetaMax, dMin, dMax, vIndices, bRight);

    return vIndices;
}

void KeyFrame::GetLineFeaturesInArea(const float thetaMin, const float thetaMax, const float dMin, const float dMax, vector<size_t>& vIndices, const bool bRight) const
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
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,vIndices,bRight);
        
        const float thetaMin2 = -M_PI_2 + std::numeric_limits<float>::epsilon(); 
        const float thetaMax2 = thetaMax;
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,vIndices,bRight);
        
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
        GetLineFeaturesInArea(thetaMin1,thetaMax1,dMin1,dMax1,vIndices,bRight);
        
        const float thetaMin2 = thetaMin; 
        const float thetaMax2 = M_PI_2 - std::numeric_limits<float>::epsilon();
        const float dMin2 = dMin; 
        const float dMax2 = dMax; 
        GetLineFeaturesInArea(thetaMin2,thetaMax2,dMin2,dMax2,vIndices,bRight);
        
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
        
    const auto& grid = (!bRight) ? mLineGrid : mLineGridRight;
    // const auto& keyLines = (NlinesLeft == -1) ? mvKeyLinesUn : 
    //                                            (!bRight) ? mvKeyLinesUn : mvKeyLinesRightUn;

    for(int ix = nMinCellDCol; ix<=nMaxCellDCol; ix++)
    {
        for(int iy = nMinCellThetaRow; iy<=nMaxCellThetaRow; iy++)
        {
            //const int iyW = Utils::Modulus(iy,LINE_THETA_GRID_ROWS); 
            const int iyW = iy;
            //const vector<size_t>& vCell = mLineGrid[ix][iyW];
            const vector<size_t>& vCell = grid[ix][iyW];
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

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        return false;
}


bool KeyFrame::UnprojectStereoLine(const int& i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd)
{
    bool res = true; 
    
    int idx = i;
    if(NlinesLeft != -1 && idx >= NlinesLeft){
        idx = mvRightToLeftLinesMatch[idx-NlinesLeft]; // get the corresponding left line if any 
        if(idx<0)
            return false;
    }    

    const float& zS = mvDepthLineStart[idx];
    const float& zE = mvDepthLineEnd[idx];
    if( (zS>0) && (zE>0) )
    {
        const float uS = mvKeyLinesUn[idx].startPointX;
        const float vS = mvKeyLinesUn[idx].startPointY;
        const float xS = (uS-cx)*zS*invfx;
        const float yS = (vS-cy)*zS*invfy;
        const Eigen::Vector3f xS3Dc(xS, yS, zS);
        
        const float uE = mvKeyLinesUn[idx].endPointX;
        const float vE = mvKeyLinesUn[idx].endPointY;
        const float xE = (uE-cx)*zE*invfx;
        const float yE = (vE-cy)*zE*invfy;
        const Eigen::Vector3f xE3Dc(xE, yE, zE);
        
        unique_lock<mutex> lock(mMutexPose);
        // const cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        // const cv::Mat Ow  = Twc.rowRange(0,3).col(3);
        //p3DStart = Rwc*xS3Dc+Ow;
        //p3DEnd   = Rwc*xE3Dc+Ow;
        p3DStart = mTwc*xS3Dc;
        p3DEnd   = mTwc*xE3Dc;
        
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
        p3DStart.setZero();
        p3DEnd.setZero();
        std::cout << "********************************************************************" << std::endl;        
        std::cout << "KeyFrame::UnprojectStereoLine() - WARNING unprojecting invalid line!" << std::endl;
        std::cout << "********************************************************************" << std::endl;
        res = false; 
    }
    
    return res; 
    
}

bool KeyFrame::UnprojectStereoLineFishEye(const int &i, Eigen::Vector3f& p3DStart, Eigen::Vector3f& p3DEnd)
{
    int idx = i;
    if(NlinesLeft != -1 && idx >= NlinesLeft){
        idx = mvRightToLeftLinesMatch[idx-NlinesLeft]; // get the corresponding left line if any 
        if(idx<0)
            return false;
    }    
    return UnprojectStereoLine(idx, p3DStart, p3DEnd); 
}


float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    vector<MapPointPtr> vpMapPoints;
    vector<MapLinePtr> vpMapLines;     
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock3(mMutexLineFeatures);     
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        vpMapLines = mvpMapLines;        
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        //if(mvpMapPoints[i])
        if(vpMapPoints[i])
        {
            //MapPointPtr& pMP = mvpMapPoints[i];            
            MapPointPtr& pMP = vpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
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
            Eigen::Vector3f x3DSw, x3DEw;
            pML->GetWorldEndPoints(x3DSw, x3DEw);            
            Eigen::Vector3f x3DMw = 0.5*(x3DSw + x3DEw);
            float z = Rcw2.dot(x3DMw)+zcw;
            vDepths.push_back(z);
        }
    }    

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    unique_lock<mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    unique_lock<mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void KeyFrame::PreSave(set<KeyFramePtr>& spKF,set<MapPointPtr>& spMP, set<GeometricCamera*>& spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the vector
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFramePtr,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;

    // Save the id of the childrens KF
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(KeyFramePtr pKFi : mspChildrens)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(KeyFrame* pKFi : mspLoopEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(KeyFramePtr pKFi : mspMergeEdges)
    {
        if(spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }

    //Camera data
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(mNextKF && spKF.find(mNextKF) != spKF.end())
        mBackupNextKFId = mNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

#if 0
void KeyFrame::PostLoad(map<long unsigned int, KeyFramePtr>& mpKFid, map<long unsigned int, MapPointPtr>& mpMPid, map<unsigned int, GeometricCamera*>& mpCamId){
    // Rebuild the empty variables

    // Pose
    SetPose(mTcw);

    mTrl = mTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    for(int i=0; i<N; ++i)
    {
        if(mvBackupMapPointsId[i] != -1)
            mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        else
            mvpMapPoints[i] = static_cast<MapPointPtr>(NULL);
    }

    // Conected KeyFrames with him weight
    mConnectedKeyFrameWeights.clear();
    for(map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
        it != end; ++it)
    {
        KeyFramePtr pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi] = it->second;
    }

    // Restore parent KeyFrame
    if(mBackupParentId>=0)
        mpParent = mpKFid[mBackupParentId];

    // KeyFrame childrens
    mspChildrens.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        mspChildrens.insert(mpKFid[*it]);
    }

    // Loop edge KeyFrame
    mspLoopEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        mspLoopEdges.insert(mpKFid[*it]);
    }

    // Merge edge KeyFrame
    mspMergeEdges.clear();
    for(vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        mspMergeEdges.insert(mpKFid[*it]);
    }

    //Camera data
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;


    // Remove all backup container
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    UpdateBestCovisibles();
}
#endif 

bool KeyFrame::ProjectPointDistort(MapPointPtr pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY = Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    // cout << "c";

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(MapPointPtr pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY= Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u = fx * PcX * invz + cx;
    v = fy * PcY * invz + cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr()
{
    unique_lock<mutex> lock(mMutexPose);
    return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
    unique_lock<mutex> lock(mMutexPose);

    return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    unique_lock<mutex> lock(mMutexPose);

    return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

Eigen::Matrix<float,3,3> KeyFrame::GetRightRotation() {
    unique_lock<mutex> lock(mMutexPose);

    return (mTrl.so3() * mTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::GetRightTranslation() {
    unique_lock<mutex> lock(mMutexPose);
    return (mTrl * mTcw).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

// template<class Archive>
// void serialize(Archive& ar, const unsigned int version)
// {
//     ar & mnId;
//     ar & const_cast<long unsigned int&>(mnFrameId);
//     ar & const_cast<double&>(mTimeStamp);
//     // Grid
//     ar & const_cast<int&>(mnGridCols);
//     ar & const_cast<int&>(mnGridRows);
//     ar & const_cast<float&>(mfGridElementWidthInv);
//     ar & const_cast<float&>(mfGridElementHeightInv);

//     // Variables of tracking
//     //ar & mnTrackReferenceForFrame;
//     //ar & mnFuseTargetForKF;
//     // Variables of local mapping
//     //ar & mnBALocalForKF;
//     //ar & mnBAFixedForKF;
//     //ar & mnNumberOfOpt;
//     // Variables used by KeyFrameDatabase
//     //ar & mnLoopQuery;
//     //ar & mnLoopWords;
//     //ar & mLoopScore;
//     //ar & mnRelocQuery;
//     //ar & mnRelocWords;
//     //ar & mRelocScore;
//     //ar & mnMergeQuery;
//     //ar & mnMergeWords;
//     //ar & mMergeScore;
//     //ar & mnPlaceRecognitionQuery;
//     //ar & mnPlaceRecognitionWords;
//     //ar & mPlaceRecognitionScore;
//     //ar & mbCurrentPlaceRecognition;
//     // Variables of loop closing
//     //serializeMatrix(ar,mTcwGBA,version);
//     //serializeMatrix(ar,mTcwBefGBA,version);
//     //serializeMatrix(ar,mVwbGBA,version);
//     //serializeMatrix(ar,mVwbBefGBA,version);
//     //ar & mBiasGBA;
//     //ar & mnBAGlobalForKF;
//     // Variables of Merging
//     //serializeMatrix(ar,mTcwMerge,version);
//     //serializeMatrix(ar,mTcwBefMerge,version);
//     //serializeMatrix(ar,mTwcBefMerge,version);
//     //serializeMatrix(ar,mVwbMerge,version);
//     //serializeMatrix(ar,mVwbBefMerge,version);
//     //ar & mBiasMerge;
//     //ar & mnMergeCorrectedForKF;
//     //ar & mnMergeForKF;
//     //ar & mfScaleMerge;
//     //ar & mnBALocalForMerge;

//     // Scale
//     ar & mfScale;
//     // Calibration parameters
//     ar & const_cast<float&>(fx);
//     ar & const_cast<float&>(fy);
//     ar & const_cast<float&>(invfx);
//     ar & const_cast<float&>(invfy);
//     ar & const_cast<float&>(cx);
//     ar & const_cast<float&>(cy);
//     ar & const_cast<float&>(mbf);
//     ar & const_cast<float&>(mb);
//     ar & const_cast<float&>(mThDepth);
//     serializeMatrix(ar, mDistCoef, version);
//     // Number of Keypoints
//     ar & const_cast<int&>(N);
//     // KeyPoints
//     serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
//     serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
//     ar & const_cast<vector<float>& >(mvuRight);
//     ar & const_cast<vector<float>& >(mvDepth);
//     serializeMatrix<Archive>(ar,mDescriptors,version);
//     // BOW
//     ar & mBowVec;
//     ar & mFeatVec;
//     // Pose relative to parent
//     serializeSophusSE3<Archive>(ar, mTcp, version);
//     // Scale
//     ar & const_cast<int&>(mnScaleLevels);
//     ar & const_cast<float&>(mfScaleFactor);
//     ar & const_cast<float&>(mfLogScaleFactor);
//     ar & const_cast<vector<float>& >(mvScaleFactors);
//     ar & const_cast<vector<float>& >(mvLevelSigma2);
//     ar & const_cast<vector<float>& >(mvInvLevelSigma2);
//     // Image bounds and calibration
//     ar & const_cast<int&>(mnMinX);
//     ar & const_cast<int&>(mnMinY);
//     ar & const_cast<int&>(mnMaxX);
//     ar & const_cast<int&>(mnMaxY);
//     ar & boost::serialization::make_array(mK_.data(), mK_.size());
//     // Pose
//     serializeSophusSE3<Archive>(ar, mTcw, version);
//     // MapPointsId associated to keypoints
//     ar & mvBackupMapPointsId;
//     // Grid
//     ar & mGrid;
//     // Connected KeyFrameWeight
//     ar & mBackupConnectedKeyFrameIdWeights;
//     // Spanning Tree and Loop Edges
//     ar & mbFirstConnection;
//     ar & mBackupParentId;
//     ar & mvBackupChildrensId;
//     ar & mvBackupLoopEdgesId;
//     ar & mvBackupMergeEdgesId;
//     // Bad flags
//     ar & mbNotErase;
//     ar & mbToBeErased;
//     ar & mbBad;

//     ar & mHalfBaseline;

//     ar & mnOriginMapId;

//     // Camera variables
//     ar & mnBackupIdCamera;
//     ar & mnBackupIdCamera2;

//     // Fisheye variables
//     ar & mvLeftToRightMatch;
//     ar & mvRightToLeftMatch;
//     ar & const_cast<int&>(NLeft);
//     ar & const_cast<int&>(NRight);
//     serializeSophusSE3<Archive>(ar, mTlr, version);
//     serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
//     ar & mGridRight;

//     // Inertial variables
//     ar & mImuBias;
//     ar & mBackupImuPreintegrated;
//     ar & mImuCalib;
//     ar & mBackupPrevKFId;
//     ar & mBackupNextKFId;
//     ar & bImu;
//     ar & boost::serialization::make_array(mVw.data(), mVw.size());
//     ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
//     ar & mbHasVelocity;
// }

template<class Archive>
void KeyFrame::serialize(Archive& ar, const unsigned int version)
{        
    using namespace boost::serialization;
    
    UNUSED_VAR(version);

    //std::cout << "saving KF " << mnId << std::endl; 
        
    ar &nNextId;
    ar &mnId;
    //ar &mpMap; 
    mbFixed = (mpMap!=0) ? (mnId==mpMap->GetInitKFidNoLock()) : (mnId==0); 

    ar & mnOriginMapId;
            
    ar & const_cast<long unsigned int&>(mnFrameId);
    ar & const_cast<double&>(mTimeStamp);
    
    // Grid related vars
    ar & const_cast<int&>(mnGridCols);
    ar & const_cast<int&>(mnGridRows);
    ar & const_cast<float&>(mfGridElementWidthInv);
    ar & const_cast<float&>(mfGridElementHeightInv);
    
    ar &const_cast< int & >(mnLineDGridCols);
    ar &const_cast< int & >(mnLineThetaGridRows);
    ar &const_cast< float & >(mfLineGridElementThetaInv);
    ar &const_cast< float & >(mfLineGridElementDInv);
    
    // Variables of tracking
    // ar & mnTrackReferenceForFrame;
    // ar & mnFuseTargetForKF;
    
    // Variables of local mapping
    // ar & mnBALocalForKF;
    // ar & mnBAFixedForKF;
    // ar & mnNumberOfOpt;
    
    // Variables used by KeyFrameDatabase
    // ar & mnLoopQuery;
    // ar & mnLoopWords;
    // ar & mLoopScore;
    // ar & mnRelocQuery;
    // ar & mnRelocWords;
    // ar & mRelocScore;
    // ar & mnMergeQuery;
    // ar & mnMergeWords;
    // ar & mMergeScore;
    // ar & mnPlaceRecognitionQuery;
    // ar & mnPlaceRecognitionWords;
    // ar & mPlaceRecognitionScore;
    // ar & mbCurrentPlaceRecognition;
    
    // Variables of loop closing
    // serializeMatrix(ar,mTcwGBA,version);    
    // serializeMatrix(ar,mTcwBefGBA,version);
    // serializeMatrix(ar,mVwbGBA,version);
    // serializeMatrix(ar,mVwbBefGBA,version);
    // ar & mTcwGBA;    
    // ar & mTcwBefGBA;
    // ar & mVwbGBA;
    // ar & mVwbBefGBA;    
    // ar & mBiasGBA;
    // ar & mnBAGlobalForKF;
        
    // Variables of Merging
    // serializeMatrix(ar,mTcwMerge,version);
    // serializeMatrix(ar,mTcwBefMerge,version);
    // serializeMatrix(ar,mTwcBefMerge,version);
    // serializeMatrix(ar,mVwbMerge,version);
    // serializeMatrix(ar,mVwbBefMerge,version);
    // ar & mTcwMerge;
    // ar & mTcwBefMerge;
    // ar & mTwcBefMerge;
    // ar & mVwbMerge;
    // ar & mVwbBefMerge;    
    // ar & mBiasMerge;
    // ar & mnMergeCorrectedForKF;
    // ar & mnMergeForKF;
    // ar & mfScaleMerge;
    // ar & mnBALocalForMerge;
    
    // Scale
    ar & mfScale;
    
    // Calibration parameters
    ar & const_cast<float&>(fx);
    ar & const_cast<float&>(fy);
    ar & const_cast<float&>(invfx);
    ar & const_cast<float&>(invfy);
    ar & const_cast<float&>(cx);
    ar & const_cast<float&>(cy);
    ar & const_cast<float&>(mbf);
    ar & const_cast<float&>(mbfInv);
    ar & const_cast<float&>(mb);
    ar & const_cast<float&>(mThDepth);
    //serializeMatrix(ar,mDistCoef,version);
    ar & const_cast< cv::Mat & >(mDistCoef);
          
    // Number of Keypoints
    ar & const_cast<int&>(N);
    // KeyPoints
    //serializeVectorKeyPoints(ar,mvKeys,version);
    //serializeVectorKeyPoints(ar,mvKeysUn,version);
    // KeyPoints, stereo coordinate and descriptors
    ar &const_cast< std::vector< cv::KeyPoint > & >(mvKeys);
    ar &const_cast< std::vector< cv::KeyPoint > & >(mvKeysUn);    
    ar &const_cast< std::vector<float>& >(mvuRight);
    ar &const_cast< std::vector<float>& >(mvDepth);
    //serializeMatrix(ar,mDescriptors,version);
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
    
    // BOW
    ar & mBowVec;
    ar & mFeatVec;
    
    // Pose relative to parent
    //serializeMatrix(ar,mTcp,version);
    // Pose relative to parent
    serializeSophusSE3<Archive>(ar, mTcp, version);
    
    // Scale related
    ar & const_cast<int&>(mnScaleLevels);
    ar & const_cast<float&>(mfScaleFactor);
    ar & const_cast<float&>(mfLogScaleFactor);
    ar & const_cast<vector<float>& >(mvScaleFactors);
    ar & const_cast<vector<float>& >(mvLevelSigma2);
    ar & const_cast<vector<float>& >(mvInvLevelSigma2);
    
    ar &const_cast< int & >(mnLineScaleLevels) &
        const_cast< float & >(mfLineScaleFactor) &
        const_cast< float & >(mfLineLogScaleFactor);
    ar &const_cast< std::vector< float > & >(mvLineScaleFactors) &
        const_cast< std::vector< float > & >(mvLineLevelSigma2) &
        const_cast< std::vector< float > & >(mvLineInvLevelSigma2);    
    
    // Image bounds and calibration
    ar & const_cast<int&>(mnMinX);
    ar & const_cast<int&>(mnMinY);
    ar & const_cast<int&>(mnMaxX);
    ar & const_cast<int&>(mnMaxY);
    ar & const_cast<int&>(mnMaxDiag);    
    //serializeMatrix(ar,mK,version);
    //ar & const_cast<cv::Mat&>(mK);  
    ar & boost::serialization::make_array(mK_.data(), mK_.size());
    if (Archive::is_loading::value) 
    {
        mK = Converter::toCvMat(mK_); 
    }
    
    // Pose
    //serializeMatrix(ar,Tcw,version);
    // mutex needed vars, but don't lock mutex in the save/load procedure
    {
        unique_lock<mutex> lock_pose(mMutexPose);
        //ar &Tcw &Twc &Ow &Cw;
        serializeSophusSE3<Archive>(ar, mTcw, version);
        // serializeSophusSE3<Archive>(ar, mTwc, version);
        // ar & boost::serialization::make_array(Ow.data(), Ow.size());
        // ar & boost::serialization::make_array(Cw.data(), Cw.size());
    }    
    if (Archive::is_loading::value) 
    {
        SetPose(mTcw);
    }
    
    {
        unique_lock<mutex> lock_feature(mMutexFeatures);
        ar &mvpMapPoints;  // hope boost deal with the pointer graph well
    }
    {
        unique_lock<mutex> lock_feature(mMutexLineFeatures);
        ar &mvpMapLines;
    }        
    // MapPointsId associated to keypoints
    //ar & mvBackupMapPointsId;
    
    // BoW
    ar &mpKeyFrameDB;
    
//    // Grid
//    ar & mGrid;
//    
//    // Connected KeyFrameWeight
//    ar & mBackupConnectedKeyFrameIdWeights;
//    
//    // Spanning Tree and Loop Edges
//    ar & mbFirstConnection;
//    ar & mBackupParentId;
//    ar & mvBackupChildrensId;
//    ar & mvBackupLoopEdgesId;
//    ar & mvBackupMergeEdgesId;
    
    {
        // Grid related
        unique_lock< mutex > lock_connection(mMutexConnections);
        ar &mGrid &mLineGrid &mConnectedKeyFrameWeights &mvpOrderedConnectedKeyFrames
            &mvOrderedWeights;
        // Spanning Tree and Loop Edges
        ar &mbFirstConnection &mpParent &mspChildrens &mspLoopEdges &mspMergeEdges;
        // Bad flags
        ar &mbNotErase &mbToBeErased &mbBad &mHalfBaseline;
    }
    // Map of Points and Lines
    ar &mpMap;
    // don't save mutex    
    
   // Bad flags
   ar & mbNotErase;
   ar & mbToBeErased;
   ar & mbBad;

   ar & mHalfBaseline;

    // Camera variables
    //ar & mnBackupIdCamera;
    //ar & mnBackupIdCamera2;
    ar & mpCamera;
    ar & mpCamera2;

    // Fisheye variables
    ar & mvLeftToRightMatch;
    ar & mvRightToLeftMatch;
    ar & const_cast<int&>(NLeft);
    ar & const_cast<int&>(NRight);
    //serializeMatrix(ar, mTlr, version);
    //serializeMatrix(ar, mTrl, version);
    serializeSophusSE3<Archive>(ar, mTlr, version);
    serializeSophusSE3<Archive>(ar, mTrl, version);    

    //serializeVectorKeyPoints(ar, mvKeysRight, version);
    ar & const_cast< std::vector< cv::KeyPoint > & >(mvKeysRight);
    ar & mGridRight;

    ar & mvLeftToRightLinesMatch;
    ar & mvRightToLeftLinesMatch;
    ar & const_cast<int&>(NlinesLeft);
    ar & const_cast<int&>(NlinesRight);    
    ar & const_cast< std::vector<cv::line_descriptor_c::KeyLine>& >(mvKeyLinesRight);
    ar & const_cast< std::vector<cv::line_descriptor_c::KeyLine>& >(mvKeyLinesRightUn);    
    ar & mLineGridRight;

    // Inertial variables
    ar & mImuBias;
    //ar & mBackupImuPreintegrated;
    ar & mpImuPreintegrated;
    ar & mImuCalib;
    //ar & mBackupPrevKFId;
    //ar & mBackupNextKFId;
    ar & mPrevKF;
    ar & mNextKF;
    
    ar & bImu;
    ar & boost::serialization::make_array(mVw.data(), mVw.size());
    ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
    ar & mbHasVelocity;    
    
    // allocate line grid in case the loaded map did not manage lines 
    if( mLineGrid.empty() )
    {
        mLineGrid.resize(mnLineDGridCols);
        for(int i=0; i<mnLineDGridCols;i++)
        {
            mLineGrid[i].resize(mnLineThetaGridRows);
        }    
    }    
    if( mLineGridRight.empty() )
    {
        mLineGridRight.resize(mnLineDGridCols);
        for(int i=0; i<mnLineDGridCols;i++)
        {
            mLineGridRight[i].resize(mnLineThetaGridRows);
        }    
    }       
}
template void KeyFrame::serialize(boost::archive::binary_iarchive &, const unsigned int);
template void KeyFrame::serialize(boost::archive::binary_oarchive &, const unsigned int);
template void KeyFrame::serialize(boost::archive::text_iarchive&, const unsigned int);
template void KeyFrame::serialize(boost::archive::text_oarchive&, const unsigned int);

} // namespace PLVS2
