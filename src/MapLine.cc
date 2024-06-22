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

#include "MapLine.h"
#include "LineMatcher.h"
#include "Utils.h"

#include<mutex>
#include <opencv2/core/base.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#define ENABLE_NEW_CHANGES 1
#define USE_ENDPOINTS_AVERAGING 0 // TODO: [Luigi] this seems to bring some problems when using IMU (futher investigation is needed)
#define DEBUG_DESTRUCTOR 0

namespace PLVS2
{

long unsigned int MapLine::nNextId=0;
mutex MapLine::mGlobalMutex;

MapLine::MapLine():
    mnFirstKFid(-1), // [Luigi]: changed from 0 to -1
    mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFramePtr>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLinePtr>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(0), muNumLineBAFailures(0)
{
    //mpReplaced = static_cast<MapLinePtr>(NULL); // Luigi: we don't need to repeat it!
}

MapLine::MapLine(const Eigen::Vector3f& PosStart, const Eigen::Vector3f& PosEnd, KeyFramePtr pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLinePtr>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), muNumLineBAFailures(0),
    mnOriginMapId(pMap->GetId())
{
    SetWorldEndPoints(PosStart, PosEnd);
    
    UdateLength();
    
    mNormalVector.setZero();

    mbTrackInViewR = false;
    mbTrackInView = false;

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const Eigen::Vector3f& PosStart, const Eigen::Vector3f& PosEnd, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFramePtr>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId()), muNumLineBAFailures(0)
{
    SetWorldEndPoints(PosStart, PosEnd); 
    
    UdateLength();
    
    Eigen::Vector3f p3DMiddle = 0.5*(mWorldPosStart+mWorldPosEnd);

    Eigen::Vector3f Ow;
    if(pFrame -> NlinesLeft == -1 || idxF < pFrame -> NlinesLeft){
        Ow = pFrame->GetCameraCenter();
    }
    else
    {
        Eigen::Matrix3f Rwl = pFrame->GetRwc();
        Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
        Eigen::Vector3f twl = pFrame->GetOw();

        Ow = Rwl * tlr + twl;
    }
    mNormalVector = p3DMiddle - Ow;
    //mNormalVector = mNormalVector/cv::norm(mNormalVector);
    mNormalVector.normalize();

//    cv::Mat PC = mWorldPosMiddle - Ow;
//    const float dist = cv::norm(PC);
    const int level = (pFrame -> NlinesLeft == -1) ? pFrame->mvKeyLinesUn[idxF].octave
                                                   : (idxF < pFrame -> NlinesLeft) ? pFrame->mvKeyLinesUn[idxF].octave
                                                                                   : pFrame -> mvKeyLinesRightUn[idxF].octave;
//    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
//    const int nLevels = pFrame->mnScaleLevels;
//    mfMaxDistance = dist*levelScaleFactor;
//    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];
    
    //const int level = pFrame->mvKeyLinesUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvLineScaleFactors[level];
    const int nLevels = pFrame->mnLineScaleLevels;

//    float distMiddle = cv::norm(p3DMiddle - Ow);
//    mfMaxDistance = distMiddle;
//    mfMinDistance = distMiddle;    
    float distStart = (PosStart - Ow).norm();
    float distEnd   = (PosEnd - Ow).norm();
    mfMaxDistance = std::max(distStart,distEnd) * levelScaleFactor;
    mfMinDistance = std::min(distStart,distEnd) * levelScaleFactor/pFrame->mvLineScaleFactors[nLevels-1];


    pFrame->mLineDescriptors.row(idxF).copyTo(mDescriptor);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const double invDepthStart, cv::Point2f uv_initStart, const double invDepthEnd, cv::Point2f uv_initEnd, KeyFramePtr pRefKF, KeyFramePtr pHostKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLinePtr>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap),
    mnOriginMapId(pMap->GetId())
{
    mInvDepthStart=invDepthStart;
    mInitUStart=(double)uv_initStart.x;
    mInitVStart=(double)uv_initStart.y;
    
    mInvDepthEnd=invDepthEnd;    
    mInitUEnd=(double)uv_initEnd.x;
    mInitVEnd=(double)uv_initEnd.y;
    
    mpHostKF = pHostKF;

    mNormalVector.setZero();

    // Worldpos is not set
    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::~MapLine()
{
#if DEBUG_DESTRUCTOR    
    std::cout << "~MapLine(): destroyed id " << mnId << std::endl;
#endif
}

void MapLine::SetWorldEndPoints(const Eigen::Vector3f &PosStart, const Eigen::Vector3f &PosEnd)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    
    mWorldPosStart = PosStart;  
    mWorldPosEnd = PosEnd;    
}

void MapLine::GetWorldEndPoints(Eigen::Vector3f &PosStart, Eigen::Vector3f &PosEnd)
{
    unique_lock<mutex> lock(mMutexPos);
    PosStart = mWorldPosStart;    
    PosEnd = mWorldPosEnd;        
}

void MapLine::GetWorldEndPointsAndLength(Eigen::Vector3f &PosStart, Eigen::Vector3f &PosEnd, float& length)
{
    unique_lock<mutex> lock(mMutexPos);
    PosStart = mWorldPosStart;    
    PosEnd = mWorldPosEnd;        
    length = mfLength;
}

void MapLine::SetWorldPosStart(const Eigen::Vector3f &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPosStart = Pos;   
}

Eigen::Vector3f MapLine::GetWorldPosStart()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosStart;
}

void MapLine::SetWorldPosEnd(const Eigen::Vector3f &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mWorldPosEnd = Pos; 
}

Eigen::Vector3f MapLine::GetWorldPosEnd()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosEnd;
}

Eigen::Vector3f MapLine::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector;
}

float MapLine::GetLength()
{
    unique_lock<mutex> lock(mMutexPos);
    return mfLength;    
}

KeyFramePtr MapLine::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapLine::AddObservation(const KeyFramePtr& pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    tuple<int,int> indexes;

    if(mObservations.count(pKF)){
        indexes = mObservations[pKF];
    }
    else{
        indexes = tuple<int,int>(-1,-1);
    }

    if(pKF -> NlinesLeft != -1 && idx >= pKF -> NlinesLeft){
        get<1>(indexes) = idx;
    }
    else{
        get<0>(indexes) = idx;
    }

    mObservations[pKF]=indexes;


    const bool bLineDepthAvailable = (!pKF->mvuRightLineStart.empty()) ;// && (!pKF->mvuRightLineEnd.empty());
    //if( !pKF->mpCamera2 && ((pKF->mvuRightLineStart[idx]>=0) && (pKF->mvuRightLineEnd[idx]>=0)) )
    if(bLineDepthAvailable && (pKF->mvuRightLineStart[idx]>=0) && (pKF->mvuRightLineEnd[idx]>=0)) // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
    {
        nObs+=2;
    }
    else
    {
        nObs++;
    }
}

void MapLine::EraseObservation(const KeyFramePtr& pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
#if !ENABLE_NEW_CHANGES
        if(mObservations.count(pKF))
        {
            //int idx = mObservations[pKF];
            tuple<int,int> indexes = mObservations[pKF];
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                //if(!pKF->mpCamera2 && ((pKF->mvuRightLineStart[leftIndex]>=0) && (pKF->mvuRightLineEnd[leftIndex]>=0)) )
                // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
                if((pKF->mvuRightLineStart[leftIndex]>=0) && (pKF->mvuRightLineEnd[leftIndex]>=0))
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(pKF);
#else  
        auto it = mObservations.find(pKF);       
        if( it != mObservations.end())
        {
            tuple<int,int> indexes  = it->second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                //if(!pKF->mpCamera2 && ((pKF->mvuRightLineStart[leftIndex]>=0) && (pKF->mvuRightLineEnd[leftIndex]>=0)) )
                // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
                if((pKF->mvuRightLineStart[leftIndex]>=0) && (pKF->mvuRightLineEnd[leftIndex]>=0))
                    nObs-=2;
                else
                    nObs--;
            }
            if(rightIndex != -1){
                nObs--;
            }

            mObservations.erase(it);         
#endif 
            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard line
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

std::map<KeyFramePtr, std::tuple<int,int>>  MapLine::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapLine::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapLine::SetBadFlag()
{
    map<KeyFramePtr, tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFramePtr, tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        int leftIndex = get<0>(mit -> second), rightIndex = get<1>(mit -> second);
        if(leftIndex != -1){
            pKF->EraseMapLineMatch(leftIndex);
        }
        if(rightIndex != -1){
            pKF->EraseMapLineMatch(rightIndex);
        }
    }

    //mpMap->EraseMapLine(this);
    mpMap->EraseMapLine(WrapPtr(this));   // N.B.: 1) we use a wrap pointer (empty deleter) since the raw ptr 'this' has not been created with the wrap pointer 
                                          //       2) the comparison operators for shared_ptr simply compare pointer values        
}

MapLinePtr MapLine::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

// replace this line with pML (this is basically used for fusing two lines)
void MapLine::Replace(MapLinePtr& pML)
{
    if(pML->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFramePtr,tuple<int,int>> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;        
        mpReplaced = pML;
        
#if USE_ENDPOINTS_AVERAGING      
        // check associations of end-points and average them 
        const Eigen::Vector3f start1 = pML->mWorldPosStart; 
        const Eigen::Vector3f end1 = pML->mWorldPosEnd; 
        Eigen::Vector3f line1Dir = start1 - end1;         
        const float normLine1Dir = line1Dir.norm();
        const float w1 = (float)pML->nObs * normLine1Dir;
        //const float w1 = normLine1Dir;
        
        Eigen::Vector3f start2 = this->mWorldPosStart; 
        Eigen::Vector3f end2 = this->mWorldPosEnd; 
        Eigen::Vector3f line2Dir = start2 - end2;       
        const float normLine2Dir = line2Dir.norm();               
        const float w2 = (float)this->nObs * normLine2Dir;     
        //const float w2 = normLine2Dir;           

        const float wTot = w1+w2;

        constexpr float minLineLength = 0.02; 
        constexpr float maxTheta = 5 * M_PI/180.0; 
        constexpr float minCosTheta = cos(maxTheta); 
        
        //if( wTot > 0) // at least one of the two lines has wi > 0
        if(normLine1Dir>minLineLength && normLine2Dir>minLineLength)
        {
            // < check line end-points association and angle between lines

            line1Dir = line1Dir/normLine1Dir;         
            line2Dir = line2Dir/normLine2Dir;   

            const float cosThetaLine = line1Dir.dot(line2Dir);

            if(fabs(cosThetaLine)>minCosTheta)
            {
                if( cosThetaLine < 0 )
                {
                    std::swap(start2, end2); // swap without changing data in the original containers 
                }
        
                const Eigen::Vector3f p3DNewStart = (start1 * w1 + start2 * w2) /wTot;
                const Eigen::Vector3f p3DNewEnd = (end1 * w1 + end2 * w2) /wTot;
                pML->mWorldPosStart = /*start1 =*/ p3DNewStart;
                pML->mWorldPosEnd = /*end1 =*/ p3DNewEnd;
            }
        }
#endif
        
    }

    for(map<KeyFramePtr,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFramePtr pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(!pML->IsInKeyFrame(pKF))
        {
            if(leftIndex != -1){
                pKF->ReplaceMapLineMatch(leftIndex, pML);
                pML->AddObservation(pKF,leftIndex);
            }
            if(rightIndex != -1){
                pKF->ReplaceMapLineMatch(rightIndex, pML);
                pML->AddObservation(pKF,rightIndex);
            }
        }
        else
        {
            if(leftIndex != -1){
                pKF->EraseMapLineMatch(leftIndex);
            }
            if(rightIndex != -1){
                pKF->EraseMapLineMatch(rightIndex);
            }
        }
    }
    pML->IncreaseFound(nfound);
    pML->IncreaseVisible(nvisible);
    pML->ComputeDistinctiveDescriptors();

    //mpMap->EraseMapLine(this);
    mpMap->EraseMapLine(WrapPtr(this));   // N.B.: 1) we use a wrap pointer (empty deleter) since the raw ptr 'this' has not been created with the wrap pointer 
                                          //       2) the comparison operators for shared_ptr simply compare pointer values    
}

bool MapLine::isBad()
{
    unique_lock<mutex> lock1(mMutexFeatures,std::defer_lock);
    unique_lock<mutex> lock2(mMutexPos,std::defer_lock);
    lock(lock1, lock2);

    return mbBad;
}

// local map lines are projected in the current frame and their visibility is checked ( Tracking::SearchLocalLines() )
void MapLine::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

// increased when found in the local map ( Tracking::TrackLocalMap() )
void MapLine::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapLine::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}


void MapLine::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFramePtr,tuple<int,int>> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;

        if(!pKF->isBad()){
            tuple<int,int> indexes = mit -> second;
            int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

            if(leftIndex != -1){
                vDescriptors.push_back(pKF->mLineDescriptors.row(leftIndex));
            }
            if(rightIndex != -1){
                vDescriptors.push_back(pKF->mLineDescriptors.row(rightIndex));
            }
        }
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            //int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            //const int distij = cv::norm(vDescriptors[i],vDescriptors[j],cv::NORM_HAMMING);
            int distij = LineMatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}


cv::Mat MapLine::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

tuple<int,int> MapLine::GetIndexInKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
#if !ENABLE_NEW_CHANGES  
    if(mObservations.count(pKF))
        return mObservations[pKF];
#else
    auto it = mObservations.find(pKF); 
    if( it != mObservations.end()) 
        return it->second;
#endif    
    else
        return tuple<int,int>(-1,-1);
}

bool MapLine::IsInKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapLine::UpdateNormalAndDepth()
{
    map<KeyFramePtr,tuple<int,int>> observations;
    KeyFramePtr pRefKF;
    Eigen::Vector3f p3DStart;
    Eigen::Vector3f p3DEnd;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        p3DStart = mWorldPosStart;
        p3DEnd   = mWorldPosEnd;
    }

    if(observations.empty())
        return;
    
    UdateLength();

    const Eigen::Vector3f p3DMiddle = 0.5*(p3DStart + p3DEnd);
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    int n=0;
    for(map<KeyFramePtr,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;

        tuple<int,int> indexes = mit -> second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        if(leftIndex != -1){
            Eigen::Vector3f Owi = pKF->GetCameraCenter();
            Eigen::Vector3f normali = p3DMiddle - Owi;
            normal = normal + normali/normali.norm();
            n++;
        }
        if(rightIndex != -1){
            Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
            Eigen::Vector3f normali = p3DMiddle - Owi;
            normal = normal + normali/normali.norm();
            n++;
        }
    }

    //Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
    //const float dist = cv::norm(PC);

    const Eigen::Vector3f Ow = pRefKF->GetCameraCenter();
    float distStart = (p3DStart - Ow).norm();
    float distEnd   = (p3DEnd - Ow).norm();
    //float distMiddle = cv::norm(p3DMiddle - Ow);

    tuple<int ,int> indexes = observations[pRefKF];
    int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
    int level;
    if(pRefKF -> NlinesLeft == -1){
        level = pRefKF->mvKeyLinesUn[leftIndex].octave;
    }
    else if(leftIndex != -1){
        level = pRefKF -> mvKeyLinesUn[leftIndex].octave;
    }
    else{
        level = pRefKF -> mvKeyLinesRightUn[rightIndex - pRefKF -> NlinesLeft].octave;
    }

    //const int level = pRefKF->mvKeyLinesUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvLineScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        //mfMaxDistance = dist*levelScaleFactor;
        //mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
	    mfMaxDistance = std::max(distStart,distEnd) * levelScaleFactor;
        mfMinDistance = std::min(distStart,distEnd) * levelScaleFactor/pRefKF->mvLineScaleFactors[nLevels-1];

        mNormalVector = normal/n;
        mNormalVector.normalize();
        //mNormalVector = mNormalVector/cv::norm(mNormalVector);
        //mNormalVectorx = cv::Matx31f(mNormalVector.at<float>(0), mNormalVector.at<float>(1), mNormalVector.at<float>(2));        
    }
}

void MapLine::UdateLength()
{
    mfLength = (mWorldPosStart - mWorldPosEnd).norm();
}

void MapLine::SetNormalVector(Eigen::Vector3f& normal)
{
    unique_lock<mutex> lock3(mMutexPos);
    mNormalVector = normal;
}

float MapLine::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMaxDistance; // 0.8 ~= 1/1.2
    //return 0.7f*mfMinDistance; // 0.7 ~= 1/1.4
}

float MapLine::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
    //return 1.4f*mfMaxDistance;
}

int MapLine::PredictScale(const float &currentDist, KeyFramePtr& pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLineLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnLineScaleLevels)
        nScale = pKF->mnLineScaleLevels-1;

    return nScale;
}

int MapLine::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLineLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnLineScaleLevels)
        nScale = pF->mnLineScaleLevels-1;

    return nScale;
}

long unsigned int MapLine::GetCurrentMaxId()
{
    //unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    return nNextId;    
}

void MapLine::PrintObservations()
{
    cout << "ML_OBS: ML " << mnId << endl;
    for(map<KeyFramePtr,tuple<int,int>>::iterator mit=mObservations.begin(), mend=mObservations.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKFi = mit->first;
        tuple<int,int> indexes = mit->second;
        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        cout << "--OBS in KF " << pKFi->mnId << " in map " << pKFi->GetMap()->GetId() << endl;
    }
}

Map* MapLine::GetMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void MapLine::UpdateMap(Map* pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

void MapLine::PreSave(set<KeyFramePtr>& spKF,set<MapLinePtr>& spML)
{
    mBackupReplacedId = -1;
    if(mpReplaced && spML.find(mpReplaced) != spML.end())
        mBackupReplacedId = mpReplaced->mnId;

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
    // Save the id and position in each KF who view it
    for(std::map<KeyFramePtr,std::tuple<int,int> >::const_iterator it = mObservations.begin(), end = mObservations.end(); it != end; ++it)
    {
        KeyFramePtr pKFi = it->first;
        if(spKF.find(pKFi) != spKF.end())
        {
            mBackupObservationsId1[it->first->mnId] = get<0>(it->second);
            mBackupObservationsId2[it->first->mnId] = get<1>(it->second);
        }
        else
        {
            EraseObservation(pKFi);
        }
    }

    // Save the id of the reference KF
    if(spKF.find(mpRefKF) != spKF.end())
    {
        mBackupRefKFId = mpRefKF->mnId;
    }
}

#if 0
void MapLine::PostLoad(map<long unsigned int, KeyFramePtr>& mpKFid, map<long unsigned int, MapLinePtr>& mpMLid)
{
    mpRefKF = mpKFid[mBackupRefKFId];
    if(!mpRefKF)
    {
        cout << "MP without KF reference " << mBackupRefKFId << "; Num obs: " << nObs << endl;
    }
    mpReplaced = static_cast<MapLinePtr>(NULL);
    if(mBackupReplacedId>=0)
    {
       map<long unsigned int, MapLinePtr>::iterator it = mpMLid.find(mBackupReplacedId);
       if (it != mpMLid.end())
        mpReplaced = it->second;
    }

    mObservations.clear();

    for(map<long unsigned int, int>::const_iterator it = mBackupObservationsId1.begin(), end = mBackupObservationsId1.end(); it != end; ++it)
    {
        KeyFramePtr pKFi = mpKFid[it->first];
        map<long unsigned int, int>::const_iterator it2 = mBackupObservationsId2.find(it->first);
        std::tuple<int, int> indexes = tuple<int,int>(it->second,it2->second);
        if(pKFi)
        {
           mObservations[pKFi] = indexes;
        }
    }

    mBackupObservationsId1.clear();
    mBackupObservationsId2.clear();
}
#endif 

template<class Archive>
void MapLine::serialize(Archive & ar, const unsigned int version)
{
    UNUSED_VAR(version);
    
    using namespace boost::serialization; 
    
    //std::cout << "saving line " << mnId << std::endl; 
    
    ar & mnId;
    ar & nNextId; // Luigi: added this 
    ar & mnFirstKFid;
    ar & mnFirstFrame;
    ar & nObs;
    // Variables used by the tracking

    // ar & mTrackProjStartX;
    // ar & mTrackProjStartY;
    // ar & mTrackStartDepth;
    // ar & mTrackStartDepthR;
    // ar & mTrackProjStartXR;
    // ar & mTrackProjStartYR;

    // ar & mTrackProjEndX;
    // ar & mTrackProjEndY;
    // ar & mTrackEndDepth;
    // ar & mTrackEndDepthR;
    // ar & mTrackProjEndXR;
    // ar & mTrackProjEndYR;

    // ar & mTrackProjMiddleX;
    // ar & mTrackProjMiddleY;
    // ar & mTrackMiddleDepth;
    // ar & mTrackMiddleDepthR;
    // ar & mTrackProjMiddleXR;
    // ar & mTrackProjMiddleYR;

    // ar & mbTrackInView;
    // ar & mbTrackInViewR;
    // ar & mnTrackScaleLevel;
    // ar & mnTrackScaleLevelR;
    // ar & mTrackViewCos;
    // ar & mTrackViewCosR;
    // ar & mnTrackReferenceForFrame;
    // ar & mnLastFrameSeen;

    // Variables used by local mapping
    // ar & mnBALocalForKF;
    // ar & mnFuseCandidateForKF;

    // // Variables used by loop closing and merging
    // ar & mnLoopPointForKF;
    // ar & mnCorrectedByKF;
    // ar & mnCorrectedReference;

    //serializeMatrix(ar,mPosStartGBA,version);
    //serializeMatrix(ar,mPosEndGBA,version);
    // ar & mPosStartGBA;
    // ar & mPosEndGBA;

    // ar & mnBAGlobalForKF;
    // ar & mnBALocalForMerge;
    
    ar & muNumLineBAFailures;    

    //serializeMatrix(ar,mPosStartMerge,version);
    //serializeMatrix(ar,mPosEndMerge,version);
    // ar & mPosStartMerge;
    // ar & mPosEndMerge;
    
    //serializeMatrix(ar,mNormalVectorMerge,version);
    // ar & mNormalVectorMerge;
    
    ar & mInvDepthStart;
    ar & mInitUStart;
    ar & mInitVStart;
    ar & mInvDepthEnd;
    ar & mInitUEnd;
    ar & mInitVEnd;

    // Protected variables
    //serializeMatrix(ar,mWorldPosStart,version);
    //serializeMatrix(ar,mWorldPosEnd,version);
    //ar & mWorldPosStart;
    ar & boost::serialization::make_array(mWorldPosStart.data(), mWorldPosStart.size());
    //ar & mWorldPosEnd;      
    ar & boost::serialization::make_array(mWorldPosEnd.data(), mWorldPosEnd.size());
    
    ar & mfLength;
    ar & mObservations;

    //ar & BOOST_SERIALIZATION_NVP(mBackupObservationsId);
    //ar & mBackupObservationsId1;
    //ar & mBackupObservationsId2;
    
    //serializeMatrix(ar,mNormalVector,version);
    //serializeMatrix(ar,mDescriptor,version);
    //ar & mNormalVector;
    ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
    
    ar & mDescriptor;
    
    ar & mpRefKF; // Luigi: added this 
    //ar & mBackupRefKFId;
    ar & mnVisible;
    ar & mnFound;

    ar & mbBad;
    ar & mpReplaced; // Luigi: added this 
    //ar & mBackupReplacedId;

    ar & mfMinDistance;
    ar & mfMaxDistance;
    ar & mpMap; // Luigi: added this 
}
template void MapLine::serialize(boost::archive::binary_iarchive &, const unsigned int);
template void MapLine::serialize(boost::archive::binary_oarchive &, const unsigned int);
template void MapLine::serialize(boost::archive::text_iarchive&, const unsigned int);
template void MapLine::serialize(boost::archive::text_oarchive&, const unsigned int);

} //namespace PLVS2
