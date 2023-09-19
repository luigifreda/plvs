/*
 * This file is part of PLVS
 * Copyright (C) 2018  Luigi Freda <luigifreda at gmail dot com>
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

#define ENABLE_NEW_CHANGES 1
#define USE_ENDPOINTS_AVERAGING 0 
#define DEBUG_DESTRUCTOR 0

namespace PLVS
{

long unsigned int MapLine::nNextId=0;
mutex MapLine::mGlobalMutex;

MapLine::MapLine(const cv::Mat& p3DStart, const cv::Mat& p3DEnd,  Map* pMap, KeyFramePtr pRefKF):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLinePtr>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap), muNumLineBAFailures(0)
{
    p3DStart.copyTo(mWorldPosStart);
    p3DEnd.copyTo(mWorldPosEnd);
    UdateLength();
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexLineCreation);
    mnId=nNextId++;
}

MapLine::MapLine(const cv::Mat& p3DStart, const cv::Mat& p3DEnd, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFramePtr>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), muNumLineBAFailures(0)
{
    p3DStart.copyTo(mWorldPosStart);
    p3DEnd.copyTo(mWorldPosEnd);
    UdateLength();
    cv::Mat p3DMiddle = 0.5*(mWorldPosStart+mWorldPosEnd);

    const cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = p3DMiddle - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

//    cv::Mat PC = mWorldPosMiddle - Ow;
//    const float dist = cv::norm(PC);
//    const int level = pFrame->mvKeysUn[idxF].octave;
//    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
//    const int nLevels = pFrame->mnScaleLevels;
//    mfMaxDistance = dist*levelScaleFactor;
//    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];
    
    const int level = pFrame->mvKeyLinesUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvLineScaleFactors[level];
    const int nLevels = pFrame->mnLineScaleLevels;

//    float distMiddle = cv::norm(p3DMiddle - Ow);
//    mfMaxDistance = distMiddle;
//    mfMinDistance = distMiddle;    
    float distStart = cv::norm(p3DStart - Ow);
    float distEnd   = cv::norm(p3DEnd - Ow);
    mfMaxDistance = std::max(distStart,distEnd) * levelScaleFactor;
    mfMinDistance = std::min(distStart,distEnd) * levelScaleFactor/pFrame->mvLineScaleFactors[nLevels-1];


    pFrame->mLineDescriptors.row(idxF).copyTo(mDescriptor);

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

void MapLine::SetWorldEndPoints(const cv::Mat &PosStart, const cv::Mat &PosEnd)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    PosStart.copyTo(mWorldPosStart);  
    PosEnd.copyTo(mWorldPosEnd);    
}

void MapLine::GetWorldEndPoints(cv::Mat &PosStart, cv::Mat &PosEnd)
{
    unique_lock<mutex> lock(mMutexPos);
    mWorldPosStart.copyTo(PosStart);    
    mWorldPosEnd.copyTo(PosEnd);        
}

void MapLine::SetWorldPosStart(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPosStart);
}

cv::Mat MapLine::GetWorldPosStart()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosStart.clone();
}

void MapLine::SetWorldPosEnd(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPosEnd);
}

cv::Mat MapLine::GetWorldPosEnd()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosEnd.clone();
}

cv::Mat MapLine::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
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

bool MapLine::AddObservation(const KeyFramePtr& pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return false;
    mObservations[pKF]=idx;

    if( (pKF->mvuRightLineStart[idx]>=0) && (pKF->mvuRightLineEnd[idx]>=0) )
        nObs+=2;
    else
        nObs++;
    return true; 
}

void MapLine::EraseObservation(const KeyFramePtr& pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
#if !ENABLE_NEW_CHANGES
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if( (pKF->mvuRightLineStart[idx]>=0) && (pKF->mvuRightLineEnd[idx]>=0) )
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);
#else            
        std::map<KeyFramePtr,size_t>::iterator it = mObservations.find(pKF);       
        if( it != mObservations.end())
        {
            const int idx = it->second;
            if( (pKF->mvuRightLineStart[idx]>=0) && (pKF->mvuRightLineEnd[idx]>=0) )
                nObs-=2;
            else
                nObs--;

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

map<KeyFramePtr, size_t> MapLine::GetObservations()
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
    map<KeyFramePtr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFramePtr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        pKF->EraseMapLineMatch(mit->second);
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
    map<KeyFramePtr,size_t> obs;
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
        cv::Mat start1 = pML->mWorldPosStart; // data sharing 
        cv::Mat end1 = pML->mWorldPosEnd; // data sharing 
        cv::Mat line1Dir = start1 - end1;         
        const float normLine1Dir = cv::norm(line1Dir);
        //const float w1 = (float)pML->nObs * normLine1Dir;
        const float w1 = normLine1Dir;
        
        cv::Mat start2 = this->mWorldPosStart; // data sharing 
        cv::Mat end2 = this->mWorldPosEnd; // data sharing 
        cv::Mat line2Dir = start2 - end2;       
        const float normLine2Dir = cv::norm(line2Dir);               
        //const float w2 = (float)this->nObs * normLine2Dir;     
        const float w2 = normLine2Dir;           

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
        
                const float wTotInv = 1.f/wTot;
                cv::Mat p3DNewStart = (start1 * w1 + start2 * w2) * wTotInv;
                cv::Mat p3DNewEnd = (end1 * w1 + end2 * w2) * wTotInv;
                p3DNewStart.copyTo( start1 );
                p3DNewEnd.copyTo( end1 );
            }
        }
#endif
        
    }

    for(map<KeyFramePtr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFramePtr pKF = mit->first;

//        if(!pML->IsInKeyFrame(pKF))
//        {
//            pML->AddObservation(pKF,mit->second); // N.B.: here we are guaranteed that the observation is added since we have preliminary checked is not in keyframe
        if(pML->AddObservation(pKF,mit->second))
        {
            pKF->ReplaceMapLineMatch(mit->second, pML);  
        }
        else
        {
            pKF->EraseMapLineMatch(mit->second);
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
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
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

    map<KeyFramePtr,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mLineDescriptors.row(mit->second));
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

int MapLine::GetIndexInKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
#if !ENABLE_NEW_CHANGES  
    if(mObservations.count(pKF))
        return mObservations[pKF];
#else
    std::map<KeyFramePtr,size_t>::iterator it = mObservations.find(pKF); 
    if( it != mObservations.end()) 
        return it->second;
#endif    
    else
        return -1;
}

bool MapLine::IsInKeyFrame(const KeyFramePtr& pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapLine::UpdateNormalAndDepth()
{
    map<KeyFramePtr,size_t> observations;
    KeyFramePtr pRefKF;
    cv::Mat p3DStart;
    cv::Mat p3DEnd;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations = mObservations;
        pRefKF = mpRefKF;
        p3DStart = mWorldPosStart.clone();
        p3DEnd   = mWorldPosEnd.clone();
    }

    if(observations.empty())
        return;
    
    UdateLength();

    const cv::Mat p3DMiddle = 0.5*(p3DStart + p3DEnd);
    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFramePtr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFramePtr pKF = mit->first;
        const cv::Mat Owi     = pKF->GetCameraCenter();
        const cv::Mat normali = p3DMiddle - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

//    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
//    const float dist = cv::norm(PC);
//    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
//    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
//    const int nLevels = pRefKF->mnScaleLevels;
    
    const cv::Mat Ow = pRefKF->GetCameraCenter();
    float distStart = cv::norm(p3DStart - Ow);
    float distEnd   = cv::norm(p3DEnd - Ow);
//  float distMiddle = cv::norm(p3DMiddle - Ow);
    
    const int level = pRefKF->mvKeyLinesUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvLineScaleFactors[level];
    const int nLevels = pRefKF->mnLineScaleLevels;    


    {
        unique_lock<mutex> lock3(mMutexPos);
//        mfMaxDistance = dist*levelScaleFactor;
//        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mfMaxDistance = std::max(distStart,distEnd) * levelScaleFactor;
        mfMinDistance = std::min(distStart,distEnd) * levelScaleFactor/pRefKF->mvLineScaleFactors[nLevels-1];
        mNormalVector = normal/n;           
        //mNormalVector = mNormalVector/cv::norm(mNormalVector);
             
    }
}

void MapLine::UdateLength()
{
    mfLength = cv::norm(mWorldPosStart - mWorldPosEnd);
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

MapLine::MapLine():
    mnFirstKFid(-1), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFramePtr>(NULL)), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapLinePtr>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(0), muNumLineBAFailures(0)
{}
template<class Archive>
void MapLine::serialize(Archive &ar, const unsigned int version) 
{
    UNUSED_VAR(version);
        
    ar & mnId & nNextId & mnFirstKFid & mnFirstFrame & nObs;
    // Tracking related vars
    ar & mTrackProjStartX;
    ar & mTrackProjStartY;
    ar & mTrackProjStartXR;
    ar & mTrackProjEndX;
    ar & mTrackProjEndY;
    ar & mTrackProjEndXR; 
    ar & mTrackProjMiddleX;
    ar & mTrackProjMiddleY;
    ar & mTrackProjMiddleXR;       
    ar & mbTrackInView;
    ar & mnTrackScaleLevel;
    ar & mTrackViewCos;
    ar & mnTrackReferenceForFrame;
    ar & mnLastFrameSeen;
    // Local Mapping related vars
    ar & mnBALocalForKF & mnFuseCandidateForKF;
    // Loop Closing related vars
    ar & mnLoopPointForKF & mnCorrectedByKF & mnCorrectedReference & mPosStartGBA & mPosEndGBA & mnBAGlobalForKF;
    ar & muNumLineBAFailures;
    // don't save the mutex
    ar & mWorldPosStart & mWorldPosEnd;
    ar & mfLength;
    ar & mObservations;
    ar & mNormalVector;
    ar & mDescriptor;
    ar & mpRefKF;
    ar & mnVisible & mnFound;
    ar & mbBad & mpReplaced;
    ar & mfMinDistance & mfMaxDistance;
    ar & mpMap;
    // don't save the mutex
}
template void MapLine::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void MapLine::serialize(boost::archive::binary_oarchive&, const unsigned int);

} //namespace PLVS
