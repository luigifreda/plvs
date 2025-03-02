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

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>

#include "LineMatcher.h"
#include "MapLine.h"
#include "Geom2DUtils.h"
#include "Logger.h"
#include "Utils.h"

#include <unordered_set>

#define VERBOSE 0

#define LOG_SEARCHES 0

#define USE_DISTSEGMENT2SEGMENT_FOR_MATCHING 0 

#define USE_STEREO_PROJECTION_CHECK 1

#define USE_REPLACE_WITH_BETTER_IN_STEREO_MATCHING 1

#define USE_MAD12 1


#if LOG_SEARCHES
#include "Logger.h"
static const string logFileName = "area_feature_search.log";
static Logger logger(logFileName);
#endif

namespace PLVS2
{


static constexpr float M_2PI = (float)(2.0 * M_PI); 

struct compare_descriptor_by_NN_dist
{
    inline bool operator()(const std::vector<cv::DMatch>& a, const std::vector<cv::DMatch>& b)
    {
        return ( a[0].distance < b[0].distance);
    }
};

struct compare_descriptor_by_NN12_dist
{
    inline bool operator()(const std::vector<cv::DMatch>& a, const std::vector<cv::DMatch>& b)
    {
        return ( a[1].distance - a[0].distance > b[1].distance - b[0].distance);
    }
};

struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const std::vector<cv::DMatch>& a, const std::vector<cv::DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};



const float LineMatcher::kDescTh = 0.1;    // parameter to avoid outliers in line matching with MAD checking

// N.B.: if the thresholds are too low lines will be not properly matched and multiple instances of a same line can be generated 
const int LineMatcher::TH_HIGH = 110;//100;//150; 
const int LineMatcher::TH_LOW = 60;//50;//80;
const int LineMatcher::TH_LOW_STEREO = 50;
const int LineMatcher::HISTO_LENGTH = 12; //30;

const float kChiSquareSegSeg = 3.84;        // value of the inverse cumulative chi-squared with 1 DOF for alpha=0.95 (Hartley-Zisserman pag 567) 
const float kChiSquareSegSegLarger = 5.024; // value of the inverse cumulative chi-squared with 1 DOF for alpha=0.975  

const float kChiSquareLineMonoProj = 5.991;        // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs for alpha=0.95  (Hartley-Zisserman pg 119)
const float kChiSquareLineMonoProjLarger = 7.378;  // chi-squared 2 2D-perpendicular-line-distances = 2 DOFs for alpha=0.975  
    
const float kChiSquareLinePointProj = 3.84;        // value of the inverse cumulative chi-squared with 1 DOF for alpha=0.95 (Hartley-Zisserman pag 567) 
const float kChiSquareLinePointProjLarger = 5.024; // value of the inverse cumulative chi-squared with 1 DOF for alpha=0.975

template <typename T>    // T can be a std container: list<int> or vector<int> 
void ComputeThreeMaxima(T* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

LineMatcher::LineMatcher(float nnratio, bool crossCheck, bool checkOrientation):mfNNratio(nnratio),mbCheckOrientation(checkOrientation)
{
    //mpBfm = std::make_shared<cv::BFMatcher>(cv::NORM_HAMMING, crossCheck); // cross-check
    
    mBdm = cv::line_descriptor_c::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
}


// used by Tracking::TrackReferenceKeyFrame(), it acts there like ORBmatcher::SearchByBoW()
// match the frame lines against the KF lines 
/// < TODO: can be possibly optimized by storing the train in the pKF without recompunting everytime (...but more memory required for KFs)
int LineMatcher::SearchByKnn(KeyFramePtr& pKF, const Frame &F, std::vector<MapLinePtr> &vpMapLineMatches)
{
    vpMapLineMatches = vector<MapLinePtr>(F.Nlines,static_cast<MapLinePtr>(NULL)); 
    
    const vector<MapLinePtr> vpMapLinesKF = pKF->GetMapLineMatches();
        
    // NOTE: recall that with fisheye cameras mLineDescriptors concatenates the left and right descriptors
    const cv::Mat& ldesc_q = pKF->mLineDescriptors; // query
    const cv::Mat& ldesc_t = F.mLineDescriptors;    // train
    
    //assert(ldesc_q.rows == vpMapLinesKF.size());
    
    list<int> rotHist[HISTO_LENGTH]; // use list since we need to keep track of elem iterators and possibly erase them (in vectors, each erase op would invalidate iterators and indexes)
    constexpr float factor = HISTO_LENGTH/M_2PI; // NOTE: kl.angle for lines is in radians and not in degrees

    cv::Mat queryMask = cv::Mat::zeros((int) vpMapLinesKF.size(), 1, CV_8UC1); 
    int numValidLinesInKF = 0; 
    for(size_t kk=0;kk<vpMapLinesKF.size();kk++)
    {
        if(!vpMapLinesKF[kk]) 
            continue;
        if(vpMapLinesKF[kk]->isBad())
            continue;
        queryMask.at<uchar>(kk)=1;
        numValidLinesInKF++;
    }
    if(numValidLinesInKF == 0) return 0; 
    
    // TODO Luigi: we could distinguish left and right matches in order to allow matching a KF line with a left F line and right F line at the same time
    std::vector<std::vector<cv::DMatch> > lmatches;
    std::vector<bool> vValidMatch;
    std::vector<bool> vbMatched(F.Nlines,false); // a "train" line can be matched to many "query" lines
    std::vector<float> vMatchDistances(F.Nlines,255);    
    std::vector<std::pair<int,list<int>::iterator>> vRotHistPairsBinIdxMatchIter(F.Nlines,std::make_pair(-1,list<int>::iterator(0)));         
            
    int numValidMatches = ComputeDescriptorMatches(ldesc_q, ldesc_t, queryMask, lmatches , vValidMatch);
    
    numValidMatches = 0; 
    for(size_t jj=0,jjEnd=lmatches.size(); jj<jjEnd; jj++)
    {
        if(!vValidMatch[jj]) continue; 
        const cv::DMatch& match = lmatches[jj][0]; // get first match from knn search 
        if( (vpMapLinesKF[match.queryIdx]) && (match.distance <= TH_LOW) )
        {
            //std::cout << "match distance : " << match.distance << std::endl; 
            if( !vbMatched[match.trainIdx] )
            {
                vbMatched[match.trainIdx] = true;
                vMatchDistances[match.trainIdx] = match.distance;                
                vpMapLineMatches[match.trainIdx] = vpMapLinesKF[match.queryIdx];
                numValidMatches++;

                if(mbCheckOrientation)
                {
                    const auto &klKF = (!pKF->mpCamera2) ? pKF->mvKeyLinesUn[match.queryIdx] :
                                                           (match.queryIdx >= pKF->NlinesLeft) ? 
                                                                pKF->mvKeyLinesRightUn[match.queryIdx - pKF->NlinesLeft] : 
                                                                pKF->mvKeyLinesUn[match.queryIdx];

                    const auto &klF = (!F.mpCamera2) ? F.mvKeyLinesUn[match.trainIdx] :
                                                       (match.trainIdx >= F.NlinesLeft) ? 
                                                            F.mvKeyLinesRightUn[match.trainIdx - F.NlinesLeft] :
                                                            F.mvKeyLinesUn[match.trainIdx];

                    //float rot = pKF->mvKeyLinesUn[match.queryIdx].angle-F.mvKeyLinesUn[match.trainIdx].angle;
                    float rot = klKF.angle-klF.angle;
                    if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI;}; // NOTE: kl.angle for lines is in radians and not in degrees
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(match.trainIdx);

                    // store the bin and iterator of the last pushed element 
                    vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));
                }                
            }
            else
            {
                if( vMatchDistances[match.trainIdx] > match.distance )
                {
                    // replace with better match
                    vMatchDistances[match.trainIdx] = match.distance;  
                    vpMapLineMatches[match.trainIdx] = vpMapLinesKF[match.queryIdx];   

                    if(mbCheckOrientation)
                    {
                        // erase the old rot match iterator 
                        auto [binIdx, it] = vRotHistPairsBinIdxMatchIter[match.trainIdx]; 
                        rotHist[binIdx].erase(it);

                        const auto &klKF = (!pKF->mpCamera2) ? pKF->mvKeyLinesUn[match.queryIdx] :
                                                              (match.queryIdx >= pKF->NlinesLeft) ? 
                                                                pKF->mvKeyLinesRightUn[match.queryIdx - pKF->NlinesLeft] : 
                                                                pKF->mvKeyLinesUn[match.queryIdx];

                        const auto &klF = (!F.mpCamera2) ? F.mvKeyLinesUn[match.trainIdx] :
                                                           (match.trainIdx >= F.NlinesLeft) ? 
                                                                F.mvKeyLinesRightUn[match.trainIdx - F.NlinesLeft] :
                                                                F.mvKeyLinesUn[match.trainIdx];

                        //float rot = pKF->mvKeyLinesUn[match.queryIdx].angle-F.mvKeyLinesUn[match.trainIdx].angle;
                        float rot = klKF.angle-klF.angle;
                        if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(match.trainIdx);

                        vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));                        
                    }                       
                }
            }            
        }
    }
    
#if VERBOSE
    std::cout << "LineMatcher::MatchByDescriptors() KF-F - matched " << numValidMatches << " lines" << std::endl;
#endif
    
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(auto it=rotHist[i].begin(); it!=rotHist[i].end(); it++)
            {
                const auto& idx = *it;
                vpMapLineMatches[idx]=static_cast<MapLinePtr>(NULL);
                numValidMatches--;
            }
        }
    }

    return numValidMatches;
}


/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 

// can be possibly used by Tracking::TrackWithMotionModel()  N.B.: at present time, it is disabled!
int LineMatcher::SearchByKnn(Frame &CurrentFrame, const Frame &LastFrame)
{
    const vector<MapLinePtr>& vpMapLinesLastFrame = LastFrame.mvpMapLines;
    
    const cv::Mat& ldesc_q = LastFrame.mLineDescriptors;    // query
    const cv::Mat& ldesc_t = CurrentFrame.mLineDescriptors; // train

    list<int> rotHist[HISTO_LENGTH]; // use list since we need to keep track of elem iterators and possibly erase them (in vectors, each erase op would invalidate iterators and indexes)
    constexpr float factor = HISTO_LENGTH/M_2PI; // NOTE: kl.angle for lines is in radians and not in degrees

    cv::Mat queryMask = cv::Mat::zeros(ldesc_q.rows, 1, CV_8UC1); 
    int numValidLinesInLastFrame = 0; 
    for(int i=0; i<LastFrame.Nlines; i++)
    {
        MapLinePtr pML = LastFrame.mvpMapLines[i];

        if(pML)
        {
            if(!LastFrame.mvbLineOutlier[i])
            {
                queryMask.at<uchar>(i)=1;
                numValidLinesInLastFrame++;
            }
        }
    }
    if(numValidLinesInLastFrame == 0 ) return 0; 
        
    std::vector<std::vector<cv::DMatch> > lmatches;
    std::vector<bool> vValidMatch;
    std::vector<bool> vbMatched(CurrentFrame.Nlines,false); // a "train" line can be matched to many "query" lines   
    std::vector<float> vMatchDistances(CurrentFrame.Nlines,255);   
    std::vector<std::pair<int,list<int>::iterator>> vRotHistPairsBinIdxMatchIter(CurrentFrame.Nlines,std::make_pair(-1,list<int>::iterator(0)));         
    
    int numValidMatches = ComputeDescriptorMatches(ldesc_q, ldesc_t, queryMask, lmatches , vValidMatch);

    numValidMatches = 0;
    for(size_t jj=0,jjEnd=lmatches.size(); jj<jjEnd; jj++)
    {
        if(!vValidMatch[jj]) continue; 
        const cv::DMatch& match = lmatches[jj][0]; // get first match from knn search 
        if( (vpMapLinesLastFrame[match.queryIdx]) && (match.distance < TH_HIGH) )
        {
            //std::cout << "match -  q:" <<  match.queryIdx << "(of "  << LastFrame.Nlines <<"), t: " << match.trainIdx << " (of " << CurrentFrame.Nlines <<") - dist: " <<match.distance << std::endl;
            if( !vbMatched[match.trainIdx] )
            {            
                vbMatched[match.trainIdx] = true;
                vMatchDistances[match.trainIdx] = match.distance;
                CurrentFrame.mvpMapLines[match.trainIdx] = vpMapLinesLastFrame[match.queryIdx];
                numValidMatches++;

                if(mbCheckOrientation)
                {
                    const auto &klCF = (!CurrentFrame.mpCamera2) ? CurrentFrame.mvKeyLinesUn[match.trainIdx] :
                                                                   (match.trainIdx >= CurrentFrame.NlinesLeft) ? 
                                                                        CurrentFrame.mvKeyLinesRightUn[match.trainIdx - CurrentFrame.NlinesLeft] :
                                                                        CurrentFrame.mvKeyLinesUn[match.trainIdx];
                    
                    const auto &klLF = (!LastFrame.mpCamera2) ? LastFrame.mvKeyLinesUn[match.queryIdx] :
                                                                (match.queryIdx >= LastFrame.NlinesLeft) ? 
                                                                    LastFrame.mvKeyLinesRightUn[match.queryIdx - LastFrame.NlinesLeft] :
                                                                    LastFrame.mvKeyLinesUn[match.queryIdx];

                    //float rot = LastFrame.mvKeyLinesUn[match.queryIdx].angle-CurrentFrame.mvKeyLinesUn[match.trainIdx].angle;
                    float rot = klLF.angle-klCF.angle;
                    if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(match.trainIdx);

                    vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));
                }                      
            }
            else
            {
                if( vMatchDistances[match.trainIdx] > match.distance )
                {
                    //std::cout << "replacing with better match " << std::endl; 
                    // replace with better match
                    vMatchDistances[match.trainIdx] = match.distance;
                    CurrentFrame.mvpMapLines[match.trainIdx] = vpMapLinesLastFrame[match.queryIdx];    

                   if(mbCheckOrientation)
                    {
                        // erase the old rot match iterator 
                        auto [binIdx, it] = vRotHistPairsBinIdxMatchIter[match.trainIdx]; 
                        rotHist[binIdx].erase(it);

                        const auto &klCF = (!CurrentFrame.mpCamera2) ? CurrentFrame.mvKeyLinesUn[match.trainIdx] :
                                                                       (match.trainIdx >= CurrentFrame.NlinesLeft) ? 
                                                                            CurrentFrame.mvKeyLinesRightUn[match.trainIdx - CurrentFrame.NlinesLeft] :
                                                                            CurrentFrame.mvKeyLinesUn[match.trainIdx];
                        
                        const auto &klLF = (!LastFrame.mpCamera2) ? LastFrame.mvKeyLinesUn[match.queryIdx] :
                                                                    (match.queryIdx >= LastFrame.NlinesLeft) ? 
                                                                        LastFrame.mvKeyLinesRightUn[match.queryIdx - LastFrame.NlinesLeft] :
                                                                        LastFrame.mvKeyLinesUn[match.queryIdx];

                        //float rot = LastFrame.mvKeyLinesUn[match.queryIdx].angle-CurrentFrame.mvKeyLinesUn[match.trainIdx].angle;
                        float rot = klLF.angle-klCF.angle;
                        if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(match.trainIdx);

                        vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));                        
                    }                       
                }
            }
        }
    }
    
#if VERBOSE    
    std::cout << "LineMatcher::MatchByDescriptors() F-F - matched " << numValidMatches << " lines" << std::endl;
#endif
    
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(auto it=rotHist[i].begin(); it!=rotHist[i].end(); it++)
            {
                const auto& idx = *it;
                CurrentFrame.mvpMapLines[idx]=static_cast<MapLinePtr>(NULL);
                numValidMatches--;
            }
        }
    }

    return numValidMatches;
}



// Used for stereo matching.
// NOTE: In the case of fisheye cameras, it is assumed to be used before frame.mLineDescriptorsRight are vconcatenated into frame.mLineDescriptors (see Frame() constructor for fisheye cameras)
// Return: A vector of matches between the left and right images. 
//         A vector (of flags) vValidMatches that is used to check if the returned match is valid or not. 
int LineMatcher::SearchStereoMatchesByKnn(Frame &frame, std::vector<cv::DMatch>& vMatches, std::vector<bool>& vValidMatches, const int& descriptorDist)
{
    MSG_ASSERT( (frame.mvKeyLinesUn.size()==frame.mvKeyLines.size()) && 
                (frame.mvKeyLinesRightUn.size()==frame.mvKeyLinesRight.size()),"Undistorted keylines should have the same size of raw ones");

    const cv::Mat& ldesc_q = frame.mLineDescriptors;      // query  (left)
    const cv::Mat& ldesc_t = frame.mLineDescriptorsRight; // train (right)
    
    cv::Mat queryMask = cv::Mat::ones(ldesc_q.rows, 1, CV_8UC1); 
        
    std::vector<std::vector<cv::DMatch> > lmatches;
    std::vector<bool> vValidDescriptorMatches;

    const size_t NLinesRight = frame.mvKeyLinesRight.size();
    
    list<int> rotHist[HISTO_LENGTH]; // use list since we need to keep track of elem iterators and possibly erase them (in vectors, each erase op would invalidate iterators and indexes)
    constexpr float factor = HISTO_LENGTH/M_2PI; // NOTE: kl.angle for lines is in radians and not in degrees

#if USE_REPLACE_WITH_BETTER_IN_STEREO_MATCHING
    std::vector<bool> vbMatched(NLinesRight,false);  // a "train" line can be matched to many "query" lines  
    std::vector<int> vStoredMatchIndex(NLinesRight,0); // vStoredMatchIndex[i] = match associated to the i-th right "train" line        
    std::vector<std::pair<int,list<int>::iterator>> vRotHistPairsBinIdxMatchIter(NLinesRight,std::make_pair(-1,list<int>::iterator(0)));         
#endif        

    int numValidMatches = ComputeDescriptorMatches(ldesc_q, ldesc_t, queryMask, lmatches, vValidDescriptorMatches);

    numValidMatches = 0;
    for(size_t jj=0,jjEnd=lmatches.size(); jj<jjEnd; jj++)
    {
        if(!vValidDescriptorMatches[jj]) continue; 
        cv::DMatch& match = lmatches[jj][0]; // get first match from knn search 
        if( match.distance < descriptorDist )
        {                   
            if(frame.mvKeyLinesUn[match.queryIdx].octave != frame.mvKeyLinesRightUn[match.trainIdx].octave) continue;
            
#if USE_REPLACE_WITH_BETTER_IN_STEREO_MATCHING           
            if( !vbMatched[match.trainIdx] )
            {
                vbMatched[match.trainIdx] = true;        

                vMatches.push_back(match);
                vValidMatches.push_back(true);                    
                numValidMatches++;

                vStoredMatchIndex[match.trainIdx] = vMatches.size()-1;                   

                if(mbCheckOrientation)
                {
                    float rot = frame.mvKeyLinesUn[match.queryIdx].angle-frame.mvKeyLinesRightUn[match.trainIdx].angle;
                    if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(vMatches.size()-1);

                    vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));
                }                    
            }
            else
            {
                const int oldMatchIdx = vStoredMatchIndex[match.trainIdx];
                cv::DMatch& oldMatch = vMatches[oldMatchIdx];
                if( oldMatch.distance > match.distance )
                {
                    // replace with better match 
                    //std::cout << "replacing with better match " << std::endl; 
                    oldMatch = match; 

                    if(mbCheckOrientation)
                    {
                        // erase the old rot match index 
                        auto [binIdx, it] = vRotHistPairsBinIdxMatchIter[match.trainIdx]; 
                        rotHist[binIdx].erase(it);

                        float rot = frame.mvKeyLinesUn[match.queryIdx].angle-frame.mvKeyLinesRightUn[match.trainIdx].angle;
                        if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(oldMatchIdx);

                        vRotHistPairsBinIdxMatchIter[match.trainIdx] = std::make_pair(bin,std::prev(rotHist[bin].end()));                        
                    }                      
                }
            }
#else
            vMatches.push_back(match);   
            vValidMatches.push_back(true);                             
            numValidMatches++;  
            if(mbCheckOrientation)
            {
                float rot = frame.mvKeyLinesUn[match.queryIdx].angle-frame.mvKeyLinesRightUn[match.trainIdx].angle;
                if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(vMatches.size()-1);
            }                          
#endif
            
        }
    }
    
#if VERBOSE    
    std::cout << "LineMatcher::MatchByDescriptors() F-F - matched " << numValidMatches << " lines" << std::endl;
#endif

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(auto it=rotHist[i].begin(); it!=rotHist[i].end(); it++)
            {
                const auto& idx = *it;
                vValidMatches[idx] = false;  
                numValidMatches--;
            }
        }
    }
    
    return numValidMatches;
}


int LineMatcher::SearchForTriangulation(KeyFramePtr& pKF1, KeyFramePtr& pKF2, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    //Compute epipole in second image
    Eigen::Vector3f C1w = pKF1->GetCameraCenter();
    Eigen::Matrix3f R2w = pKF2->GetRotation();
    Eigen::Vector3f t2w = pKF2->GetTranslation();
    Eigen::Vector3f C12  = R2w*C1w+t2w;

    // undistorted projection of epipole in second image (used with pinhole cameras and mono observations)
    const float invz = 1.0f/C12(2);
    const float ex = pKF2->fx*C12(0)*invz+pKF2->cx;
    const float ey = pKF2->fy*C12(1)*invz+pKF2->cy;  
    
    GeometricCamera *pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

#if 0
    Sophus::SE3f T12;
    Sophus::SE3f Tll, Tlr, Trl, Trr;
    Eigen::Matrix3f R12; // for fastest computation
    Eigen::Vector3f t12; // for fastest computation

    if(!pKF1->mpCamera2 && !pKF2->mpCamera2){
        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();
    }
    else{
        const Sophus::SE3f Tr1w = pKF1->GetRightPose();
        const Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
        Tll = T1w * Tw2;
        Tlr = T1w * Twr2;
        Trl = Tr1w * Tw2;
        Trr = Tr1w * Twr2;
    }

    const Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr  = Tlr.rotationMatrix(), Rrl  = Trl.rotationMatrix(), Rrr  = Trr.rotationMatrix();
    const Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();
#endif 

    int nmatches=0;

    const vector<MapLinePtr> vpMapLinesKF1 = pKF1->GetMapLineMatches();
    const vector<MapLinePtr> vpMapLinesKF2 = pKF2->GetMapLineMatches();
        
    const cv::Mat& ldesc_q = pKF1->mLineDescriptors; // query
    const cv::Mat& ldesc_t = pKF2->mLineDescriptors; // train

    std::vector<bool> vbMatched2(pKF2->Nlines,false); // a "train" line can be matched to many "query" lines
    std::vector<float> vMatched2Distances(pKF2->Nlines,255);   
    std::vector<int> vMatches2(pKF2->Nlines,-1);    // vMatches2[i] = line of pKF1 matched to the i-th line of pKF2

    std::vector<bool> vbStereo1(pKF1->Nlines,false);        
            
    //assert(ldesc_q.rows == vpMapLinesKF.size());
    
    list<int> rotHist[HISTO_LENGTH]; // use list since we need to keep track of elem iterators and possibly erase them (in vectors, each erase op would invalidate iterators and indexes)
    constexpr float factor = HISTO_LENGTH/M_2PI; // NOTE: kl.angle for lines is in radians and not in degrees

    cv::Mat queryMask = cv::Mat::zeros((int) vpMapLinesKF1.size(), 1, CV_8UC1); 
    int numLinesToMatchInKF = 0; 
    for(size_t idx1=0,idx1End=vpMapLinesKF1.size();idx1<idx1End;idx1++)
    {
        MapLinePtr pML1 = vpMapLinesKF1[idx1];   
        
        // If there is already a MapLine skip
        if(pML1) 
            continue;
        
        // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if > 0 they signal the availability of the depths.  
        const bool bStereo1 = (!pKF1->mvuRightLineStart.empty()) && (pKF1->mvuRightLineStart[idx1]>=0) && (pKF1->mvuRightLineEnd[idx1]>=0);
        vbStereo1[idx1] = bStereo1; 

        if(bOnlyStereo)
            if(!bStereo1)
                continue;            

        queryMask.at<uchar>(idx1)=1;
        numLinesToMatchInKF++;
    }
    //std::cout << "num lines to match: " << numLinesToMatchInKF << std::endl; 
    if(numLinesToMatchInKF == 0) return 0; 
    
    std::vector<std::vector<cv::DMatch>> lmatches;
    std::vector<bool> vValidMatch;

    std::vector<std::pair<int,list<int>::iterator>> vRotHistPairsBinIdxMatchIter(pKF2->Nlines,std::make_pair(-1,list<int>::iterator(0)));             
            
    int numValidMatches = ComputeDescriptorMatches(ldesc_q, ldesc_t, queryMask, lmatches, vValidMatch);
    //std::cout << "num valid matches: " << numValidMatches << std::endl; 
    
    nmatches = 0; 
    for(size_t jj=0,jjEnd=lmatches.size(); jj<jjEnd; jj++)
    {
        if(!vValidMatch[jj]) continue; 
        const cv::DMatch& match = lmatches[jj][0]; // get first match from knn search 
        
        if(match.distance > TH_LOW) continue;
                                            
        const int idx1 = match.queryIdx;
        const int idx2 = match.trainIdx;
        
        MapLinePtr pML2 = vpMapLinesKF2[idx2];

        // If we have already matched or there is a MapLine skip
        if(pML2)
            continue;

        // NOTE: With fisheye cameras, mvuRightLineStart and mvuRightLineEnd values cannot be directly used, however if >0 they signal the availability of the depths.  
        const bool bStereo2 = (!pKF2->mvuRightLineStart.empty()) && (pKF2->mvuRightLineStart[idx2]>=0) && (pKF2->mvuRightLineEnd[idx2]>=0);

        if(bOnlyStereo)
            if(!bStereo2)
                continue;
        
        const bool bStereo1 = vbStereo1[idx1];


        const auto &kl1 = (!pKF1->mpCamera2) ? pKF1->mvKeyLinesUn[idx1] :
                                              (idx1 >= pKF1->NlinesLeft) ? 
                                                pKF1->mvKeyLinesRightUn[idx1 - pKF1->NlinesLeft] : 
                                                pKF1->mvKeyLinesUn[idx1];

        const auto &kl2 = (!pKF2->mpCamera2) ? pKF2->mvKeyLinesUn[idx2] :
                                              (idx2 >= pKF2->NlinesLeft) ? 
                                                pKF2->mvKeyLinesRightUn[idx2 - pKF2->NlinesLeft] : 
                                                pKF2->mvKeyLinesUn[idx2];

        //const bool bRight1 = (pKF1->NlinesLeft == -1 || idx1 < pKF1->NlinesLeft) ? false : true;
        //const bool bRight2 = (pKF2->NlinesLeft == -1 || idx2 < pKF1->NlinesLeft) ? false : true;

        if(!bStereo1 && !bStereo2 && !pKF1->mpCamera2)
        {
            // if monocular observation check if we are too close to epipole 

            //const cv::line_descriptor_c::KeyLine& kl2 = pKF2->mvKeyLinesUn[idx2];      
                                       
            //const float xm = 0.5*(kl2.startPointX + kl2.endPointX);
            //const float ym = 0.5*(kl2.startPointY + kl2.endPointY);
            const float disteSx = ex-kl2.startPointX;
            const float disteSy = ey-kl2.startPointY;

            const float disteEx = ex-kl2.endPointX;
            const float disteEy = ey-kl2.endPointY;

            const float disteTh = 100*pKF2->mvLineScaleFactors[kl2.octave];
            if( (disteSx*disteSx+disteSy*disteSy<disteTh) || (disteEx*disteEx+disteEy*disteEy<disteTh) )
                continue;
        }        
                    
        if( !vbMatched2[idx2] )
        {
            vbMatched2[idx2] = true;
            vMatched2Distances[idx2] = match.distance;                
            vMatches2[idx2] = idx1;
            //std::cout << "match: (" << idx1 << ", " << idx2 << ")  dist: " << match.distance << std::endl;
            nmatches++;

            if(mbCheckOrientation)
            {
                //                             query                          train
                //float rot = pKF1->mvKeyLinesUn[idx1].angle-pKF2->mvKeyLinesUn[idx2].angle;
                float rot = kl1.angle-kl2.angle;
                if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(idx2); // push train

                vRotHistPairsBinIdxMatchIter[idx2] = std::make_pair(bin,std::prev(rotHist[bin].end()));
            }                
        }
        else
        {
            if( vMatched2Distances[idx2] > match.distance )
            {
                //std::cout << "replacing with better match " << match.distance << std::endl; 
                // replace with better match  
                vMatched2Distances[idx2] = match.distance;  
                vMatches2[idx2] = idx1;
                //std::cout << "match: (" << idx1 << ", " << idx2 << ")  dist: " << match.distance << std::endl;    

                if(mbCheckOrientation)
                {
                    // erase the old rot match iterator 
                    auto [binIdx, it] = vRotHistPairsBinIdxMatchIter[idx2]; 
                    rotHist[binIdx].erase(it);

                    //                             query                          train
                    //float rot = pKF1->mvKeyLinesUn[idx1].angle-pKF2->mvKeyLinesUn[idx2].angle;
                    float rot = kl1.angle-kl2.angle;
                    if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(idx2); // push train

                    vRotHistPairsBinIdxMatchIter[idx2] = std::make_pair(bin,std::prev(rotHist[bin].end()));                     
                }                       
            }
        }            
        
    }
    
#if VERBOSE
    std::cout << "LineMatcher::MatchByDescriptors() KF-KF - matched " << numValidMatches << " lines" << std::endl;
#endif
    

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(auto it=rotHist[i].begin(); it!=rotHist[i].end(); it++)
            {
                const auto& idx2 = *it;
                vMatches2[idx2]=-1;
                nmatches--;
            }
        }
    }

    // prepare the output     
    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches2.size(); i<iend; i++)
    {
        if(vMatches2[i]<0)
            continue;
        //                                    idx1     idx2
        vMatchedPairs.push_back(make_pair(vMatches2[i],i)); // vMatches2[i] = line of pKF1 matched to the i-th line of pKF2
    }

    return nmatches;
}


// guided search by probjection; used by Tracking::TrackWithMotionModel()
int LineMatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const bool bLargerSearch, const bool bMono)
{
    int nmatches = 0;

    const float thChiSquareLineMonoProj = bLargerSearch ? kChiSquareLineMonoProjLarger : kChiSquareLineMonoProj; 
    const float thChiSquareSegSeg = bLargerSearch ? kChiSquareSegSegLarger : kChiSquareSegSeg;
    const float thChiSquareLinePointProj = bLargerSearch ? kChiSquareLinePointProjLarger : kChiSquareLinePointProj;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(100);
    constexpr float factor = HISTO_LENGTH/M_2PI;  // NOTE: kl.angle for lines is in radians and not in degrees

    const Sophus::SE3f Tcw = CurrentFrame.GetPose();
    const Eigen::Matrix3f Rcw = CurrentFrame.GetRcw();
    const Eigen::Vector3f tcw = Tcw.translation();
    const Eigen::Vector3f twc = Tcw.inverse().translation();

    const Sophus::SE3f Tlw = LastFrame.GetPose();
    const Eigen::Vector3f tlc = Tlw * twc;

    const bool bForward = tlc(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc(2)>CurrentFrame.mb && !bMono;

    LineProjection proj;
    const FrameTransformsForLineProjection currFrameTransforms(CurrentFrame);
    Line2DRepresentation projLineRepresentation;
    Line2DRepresentation projRightLineRepresentation;

#if LOG_SEARCHES    
    logger << "\n\n\n***********************************************************\n\n\n" << std::endl;
    logger << "matching frame: " << CurrentFrame.mnId << " against frame: " << LastFrame.mnId << std::endl; 
#endif
            
    for(int i=0; i<LastFrame.Nlines; i++)
    {
        MapLinePtr pML = LastFrame.mvpMapLines[i];

        if(pML)
        {
            if(!LastFrame.mvbLineOutlier[i]) 
            {
            //     Project
            //    cv::Mat x3Dw = pML->GetWorldPos();
            //    cv::Mat x3Dc = Rcw*x3Dw+tcw;

            //    const float xc = x3Dc.at<float>(0);
            //    const float yc = x3Dc.at<float>(1);
            //    const float invzc = 1.0/x3Dc.at<float>(2);

            //    if(invzc<0)
            //        continue;

            //    float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
            //    float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

            //    if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
            //        continue;
            //    if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
            //        continue;
                
                // project line to current frame 
                //if(!proj.ProjectLineWithCheck(Rcw, tcw, pML, CurrentFrame))
		        if(!proj.ProjectLineWithCheck(pML, CurrentFrame, currFrameTransforms))
                    continue;
                            
                // this can be a left or right line                            
                const auto &klLF = (!LastFrame.mpCamera2) ? LastFrame.mvKeyLinesUn[i] :
                                                            (i >= LastFrame.NlinesLeft) ? 
                                                                LastFrame.mvKeyLinesRightUn[i - LastFrame.NlinesLeft] :
                                                                LastFrame.mvKeyLinesUn[i];                                                        
                int nLastOctave = klLF.octave;

            //    // Search in a window. Size depends on scale
            //    float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;
                Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation);  
             
            //    if(bForward)
            //        vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
            //    else if(bBackward)
            //        vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
            //    else
            //        vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                const float scale = CurrentFrame.mvLineScaleFactors[nLastOctave];
                const float deltaTheta = Frame::kDeltaTheta*scale;
                const float deltaD = Frame::kDeltaD*scale;
                if(bForward) 
                    vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nLastOctave);                
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, 0, nLastOctave);                   
                else
                    vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nLastOctave-1, nLastOctave+1);     
              
                
#if LOG_SEARCHES
                                   
                logger << "line "<< i <<" - theta: " << projLineRepresentation.theta*180/M_PI << ", d: " << projLineRepresentation.d << std::endl;
                logger << "found:" << std::endl;
                for(size_t jj=0; jj<vIndices2.size(); jj++)
                {
                    //const cv::line_descriptor_c::KeyLine& currentFrameLine = CurrentFrame.mvKeyLinesUn[vIndices2[jj]];
                    const auto &klCF = (!CurrentFrame.mpCamera2) ? CurrentFrame.mvKeyLinesUn[vIndices2[jj]] :
                                                                (vIndices2[jj] >= CurrentFrame.NlinesLeft) ? 
                                                                        CurrentFrame.mvKeyLinesRightUn[vIndices2[jj] - CurrentFrame.NlinesLeft] :
                                                                        CurrentFrame.mvKeyLinesUn[vIndices2[jj]];                     
                    Line2DRepresentation lineRepresentation;
                    Geom2DUtils::GetLine2dRepresentation(klCF.startPointX, klCF.startPointY, 
                                                         klCF.endPointX, klCF.endPointY, lineRepresentation);
                    logger << "line " << vIndices2[jj] <<", "<< lineRepresentation.theta*180/M_PI << ", " << lineRepresentation.d << std::endl;
                }
                logger << "-------------------------------------------------" << std::endl;
#endif
                
                if(vIndices2.empty())
                    continue;

                const cv::Mat dML = pML->GetDescriptor();

                int bestDist  = 256;
                int bestIdx  = -1;
                int bestDist2 = 256;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit; // this is a left line index 
                    if(CurrentFrame.mvpMapLines[i2])
                        if(CurrentFrame.mvpMapLines[i2]->Observations()>0)
                            continue;

                //    if(CurrentFrame.mvuRight[i2]>0)
                //    {
                //        const float ur = u - CurrentFrame.mbf*invzc;
                //        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                //        if(er>radius)
                //            continue;
                //    }

                    const cv::line_descriptor_c::KeyLine& currentFrameLine = CurrentFrame.mvKeyLinesUn[i2]; // i2 belongs to vIndices2 that contains left line indices 
                    //MSG_ASSERT(i2<CurrentFrame.NlinesLeft,"This must be a left line");
                                
#if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING                                
                    // dist segment-segment 
                    const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(proj.uS,proj.vS), Eigen::Vector2f(proj.uE,proj.vE), 
                                                                              Eigen::Vector2f(currentFrameLine.startPointX, currentFrameLine.startPointY),  
                                                                              Eigen::Vector2f(currentFrameLine.endPointX, currentFrameLine.endPointY) );

                    if(distSegSeg*distSegSeg*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareSegSeg)  
                    {
                        continue;
                    }
#else 
                    // distance point-line + point-line
                    // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                    // constraint: 0 = l1*u1 + l2*v1 + l3  
                    const float distS = projLineRepresentation.nx*currentFrameLine.startPointX + projLineRepresentation.ny*currentFrameLine.startPointY - projLineRepresentation.d;
                    const float distE = projLineRepresentation.nx*currentFrameLine.endPointX + projLineRepresentation.ny*currentFrameLine.endPointY - projLineRepresentation.d;
                    //const float err2 = distS*distS + distE*distE; 
                    const float distS2 = distS*distS;
                    const float distE2 = distE*distE;

                    //if(err2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLineMonoProj)  
                    if(distS2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj 
                    || distE2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj)
                    {
                        continue;
                    }                                                         
#endif                     

#if USE_STEREO_PROJECTION_CHECK
                    // fast stereo projection check with rectified images 
                    if(!CurrentFrame.mpCamera2 && (!CurrentFrame.mvuRightLineStart.empty()) && CurrentFrame.mvuRightLineStart[i2]>=0 && CurrentFrame.mvuRightLineEnd[i2]>=0)
                    {
                        // we assume left and right images are rectified 

                        // get right projection of pML 
                        const float proj_uSr = proj.uS - CurrentFrame.mbf*proj.invSz;
                        const float proj_vSr = proj.vS; 

                        const float proj_uEr = proj.uE - CurrentFrame.mbf*proj.invEz;
                        const float proj_vEr = proj.vE;  

                        Geom2DUtils::GetLine2dRepresentation(proj_uSr, proj_vSr, proj_uEr, proj_vEr, projRightLineRepresentation);  

                        // get current frame line end-points on right image
                        const float uSr = CurrentFrame.mvuRightLineStart[i2];
                        const float vSr = currentFrameLine.startPointY;   

                        const float uEr = CurrentFrame.mvuRightLineEnd[i2];
                        const float vEr = currentFrameLine.endPointY;                                                       

                        // distance point-line + point-line on right line
                        // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                        // constraint: 0 = l1*u1 + l2*v1 + l3  
                        const float distSr = projRightLineRepresentation.nx*uSr + projRightLineRepresentation.ny*vSr - projRightLineRepresentation.d;
                        const float distEr = projRightLineRepresentation.nx*uEr + projRightLineRepresentation.ny*vEr - projRightLineRepresentation.d;
                        //const float err2r = distSr*distSr + distEr*distEr; 
                        const float distSr2 = distSr*distSr;
                        const float distEr2 = distEr*distEr;

                        //if(err2r*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLineMonoProj)  
                        if(distSr2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj 
                        || distEr2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj)
                        {
                            continue;
                        }                
                    }  
#endif  
                    const cv::Mat &d = CurrentFrame.mLineDescriptors.row(i2); 

                    const int dist = DescriptorDistance(dML,d);

                    
                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestIdx=i2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                    
                }

                if( (bestDist<=TH_HIGH) && (bestIdx>=0) )
                {
                    
                    if(bestDist>mfNNratio*bestDist2)
                        continue;    
                                
                    //std::cout << "match distance H: " << bestDist << std::endl; 
                    
                    CurrentFrame.mvpMapLines[bestIdx]=pML;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        const auto &klCF = CurrentFrame.mvKeyLinesUn[bestIdx]; // bestIdx belongs to vIndices2 that contains left line indices 
                        //MSG_ASSERT(bestIdx<CurrentFrame.NlinesLeft,"This must be a left line");

                        //float rot = LastFrame.mvKeyLinesUn[i].angle-CurrentFrame.mvKeyLinesUn[bestIdx].angle;
                        float rot = klLF.angle-klCF.angle;
                        if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx);
                    }
                }

                if(CurrentFrame.NlinesLeft != -1)
                {
                    // match on right image 

                    if(!proj.ProjectLineWithCheck(pML, CurrentFrame, currFrameTransforms, true/*bRight*/))
                        continue;

                    vector<size_t> vIndices2;
                    Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation);

                    if(bForward) 
                        vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nLastOctave, Frame::kMaxInt, true /*bRight*/);                
                    else if(bBackward)
                        vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, 0, nLastOctave, true /*bRight*/);                   
                    else
                        vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nLastOctave-1, nLastOctave+1, true /*bRight*/);                           
                
                    if(vIndices2.empty())
                        continue;

                    int bestDist  = 256;
                    int bestIdx  = -1;
                    int bestDist2 = 256;

                    for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                    {
                        const size_t i2 = *vit; // this is a right line index is in [0,...,NlinesRight-1]
                        if(CurrentFrame.mvpMapLines[i2+CurrentFrame.NlinesLeft])
                            if(CurrentFrame.mvpMapLines[i2+CurrentFrame.NlinesLeft]->Observations()>0)
                                continue;

                        const auto &currentFrameLine = CurrentFrame.mvKeyLinesRightUn[i2]; // NOTE: here we don't need +CurrentFrame.NlinesLeft !
                        //const int &klLevel= currentFrameLine.octave;
                                    
    #if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING                                
                        // dist segment-segment 
                        const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(proj.uS,proj.vS), Eigen::Vector2f(proj.uE,proj.vE), 
                                                                                Eigen::Vector2f(currentFrameLine.startPointX, currentFrameLine.startPointY),  
                                                                                Eigen::Vector2f(currentFrameLine.endPointX, currentFrameLine.endPointY) );

                        if(distSegSeg*distSegSeg*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareSegSeg)  
                        {
                            continue;
                        }
    #else 
                        // distance point-line + point-line
                        // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                        // constraint: 0 = l1*u1 + l2*v1 + l3  
                        const float distS = projLineRepresentation.nx*currentFrameLine.startPointX + projLineRepresentation.ny*currentFrameLine.startPointY - projLineRepresentation.d;
                        const float distE = projLineRepresentation.nx*currentFrameLine.endPointX + projLineRepresentation.ny*currentFrameLine.endPointY - projLineRepresentation.d;
                        //const float err2 = distS*distS + distE*distE; 
                        const float distS2 = distS*distS;
                        const float distE2 = distE*distE;

                        //if(err2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLineMonoProj)  
                        if(distS2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj || 
                           distE2*(CurrentFrame.mvLineInvLevelSigma2[nLastOctave])>thChiSquareLinePointProj)
                        {
                            continue;
                        }                                                         
    #endif                     

                        const cv::Mat &d = CurrentFrame.mLineDescriptors.row(i2+CurrentFrame.NlinesLeft); 

                        const int dist = DescriptorDistance(dML,d);

                        
                        if(dist<bestDist)
                        {
                            bestDist2=bestDist;
                            bestDist=dist;
                            bestIdx=i2;
                        }
                        else if(dist<bestDist2)
                        {
                            bestDist2=dist;
                        }
                        
                    }

                    if( (bestDist<=TH_HIGH) && (bestIdx>=0) )
                    {
                        
                        if(bestDist>mfNNratio*bestDist2)
                            continue;    
                                    
                        //std::cout << "match distance H: " << bestDist << std::endl; 
                        
                        CurrentFrame.mvpMapLines[bestIdx+CurrentFrame.NlinesLeft]=pML;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            const auto &klCF = CurrentFrame.mvKeyLinesRightUn[bestIdx];

                            float rot = klLF.angle-klCF.angle;
                            if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdx);
                        }
                    }
                }
            }
        }
    }

    //Apply rotation consistency
   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

       for(int i=0; i<HISTO_LENGTH; i++)
       {
           if(i!=ind1 && i!=ind2 && i!=ind3)
           {
               for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
               {
                   CurrentFrame.mvpMapLines[rotHist[i][j]]=static_cast<MapLinePtr>(NULL);
                   nmatches--;
               }
           }
       }
   }

#if VERBOSE    
    std::cout << "LineMatcher::MatchByProjection() F-F - matched " << nmatches << " lines" << std::endl;    
#endif
    return nmatches;
}



/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 

// used by Tracking::SearchLocalLines()  N.B.: at present time is disabled!
int LineMatcher::SearchByKnn(Frame &F, const std::vector<MapLinePtr> &vpMapLines)
{      
    if(vpMapLines.empty()) return 0; 
    
    cv::Mat ldesc_q   = cv::Mat( (int) vpMapLines.size(), (int)32, (int)CV_8UC1, cv::Scalar(0)); // query
    cv::Mat queryMask = cv::Mat::zeros(ldesc_q.rows, 1, CV_8UC1); 
    int numValidLines = 0; 
    for(size_t kk=0,kkEnd=vpMapLines.size();kk<kkEnd;kk++)
    {
        if(!vpMapLines[kk]) 
            continue;
        if(!vpMapLines[kk]->mbTrackInView && !vpMapLines[kk]->mbTrackInViewR)
            continue;
        if(vpMapLines[kk]->isBad())
            continue;
        vpMapLines[kk]->GetDescriptor().copyTo(ldesc_q.row(kk));
        queryMask.at<uchar>(kk)=1;
        numValidLines++;
    }
    if(numValidLines == 0) return 0; 
          
    const cv::Mat& ldesc_t = F.mLineDescriptors;    // train
        
    std::vector<std::vector<cv::DMatch> > lmatches;
    std::vector<bool> vValidMatch;
            
    int numValidMatches = ComputeDescriptorMatches(ldesc_q, ldesc_t, queryMask, lmatches , vValidMatch);

    numValidMatches = 0;
    for(size_t jj=0,jjEnd=lmatches.size(); jj<jjEnd; jj++)
    {
        if(!vValidMatch[jj]) continue; 
        const cv::DMatch& match = lmatches[jj][0]; // get first match from knn search 
        //std::cout << "match -  q:" <<  match.queryIdx << "(of "  << LastFrame.Nlines <<"), t: " << match.trainIdx << " (of " << CurrentFrame.Nlines <<")" << std::endl;
        if( (vpMapLines[match.queryIdx]) && (match.distance < TH_HIGH))
        {
            F.mvpMapLines[match.trainIdx] = vpMapLines[match.queryIdx];
            numValidMatches++;
        }
    }
    
#if VERBOSE    
    std::cout << "LineMatcher::MatchByDescriptors() F-Map - matched " << numValidMatches << " lines" << std::endl;
#endif
    
    return numValidMatches;    
}


// used by Tracking::SearchLocalLines()
// N.B: vpMapLines have already being projected on the current frame 
int LineMatcher::SearchByProjection(Frame &F, const std::vector<MapLinePtr> &vpMapLines, const bool bLargerSearch)
{
    const float thChiSquareLineMonoProj = bLargerSearch ? kChiSquareLineMonoProjLarger : kChiSquareLineMonoProj; 
    const float thChiSquareSegSeg = bLargerSearch ? kChiSquareSegSegLarger : kChiSquareSegSeg;
    const float thChiSquareLinePointProj = bLargerSearch ? kChiSquareLinePointProjLarger : kChiSquareLinePointProj;

    int nmatches=0;

//    const bool bFactor = th!=1.0;

    LineProjection proj;
    Line2DRepresentation projLineRepresentation;
    Line2DRepresentation projRightLineRepresentation;
    
    for(size_t iML=0,iMLEnd=vpMapLines.size(); iML<iMLEnd; iML++)
    {
        MapLinePtr pML = vpMapLines[iML];
        if(!pML->mbTrackInView && !pML->mbTrackInViewR)
            continue;

        if(pML->isBad())
            continue;

        if(pML->mbTrackInView)
        {
            // check of left image 

            const int &nPredictedLevel = pML->mnTrackScaleLevel;
    
        //    // The size of the window will depend on the viewing direction
        //    float r = RadiusByViewingCos(pML->mTrackViewCos);
    
        //    if(bFactor)
        //        r*=th;

            //const vector<size_t> vIndices = F.GetFeaturesInArea(pML->mTrackProjX,pML->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);
            
            Geom2DUtils::GetLine2dRepresentation(pML->mTrackProjStartX, pML->mTrackProjStartY, pML->mTrackProjEndX, pML->mTrackProjEndY, projLineRepresentation); 
            
            const float scale = F.mvLineScaleFactors[nPredictedLevel];
            const float deltaTheta = Frame::kDeltaTheta*scale;
            const float deltaD = Frame::kDeltaD*scale;        
            const vector<size_t> vIndices = F.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nPredictedLevel-1,nPredictedLevel);

            if(vIndices.empty())
                continue;

            const cv::Mat MPdescriptor = pML->GetDescriptor();

            int bestDist  = 256;
            int bestLevel= -1;
            int bestDist2 = 256;
            int bestLevel2 = -1;
            int bestIdx = -1;

            // Get best and second matches with near keylines
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)   
            {
                const size_t idx = *vit; // this is a left line index

                if(F.mvpMapLines[idx])
                    if(F.mvpMapLines[idx]->Observations()>0)
                        continue;

            //    if(F.mvuRight[idx]>0)
            //    {
            //        const float er = fabs(pML->mTrackProjStartXR-F.mvuRight[idx]);
            //        if(er>r*F.mvScaleFactors[nPredictedLevel])
            //            continue;
            //    }

                const cv::line_descriptor_c::KeyLine& kl = F.mvKeyLinesUn[idx];
                MSG_ASSERT(idx < F.NlinesLeft,"This must be a left line");              
                
    #if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING             
                // dist segment-segment 
                const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(pML->mTrackProjStartX, pML->mTrackProjStartY), 
                                                                            Eigen::Vector2f(pML->mTrackProjEndX, pML->mTrackProjEndY),
                                                                            Eigen::Vector2f(kl.startPointX, kl.startPointY),
                                                                            Eigen::Vector2f(kl.endPointX, kl.endPointY));

                if(distSegSeg*distSegSeg*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareSegSeg) 
                {
                    continue;
                }
    #else 
                // distance point-line + point-line
                // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                // constraint: 0 = l1*u1 + l2*v1 + l3  
                const float distS = projLineRepresentation.nx*kl.startPointX + projLineRepresentation.ny*kl.startPointY - projLineRepresentation.d;
                const float distE = projLineRepresentation.nx*kl.endPointX + projLineRepresentation.ny*kl.endPointY - projLineRepresentation.d;
                //const float err2 = distS*distS + distE*distE; 
                const float distS2 = distS*distS;
                const float distE2 = distE*distE;

                //if(err2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLineMonoProj)  
                if(distS2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj || 
                   distE2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj)
                {
                    continue;
                }    
    #endif 


    #if USE_STEREO_PROJECTION_CHECK
                // fast stereo projection check with rectified images    
                if(!F.mpCamera2 && (!F.mvuRightLineStart.empty()) &&F.mvuRightLineStart[idx]>=0 && F.mvuRightLineEnd[idx]>=0)
                {
                    // we assume left and right images are rectified 

                    // get right projection of pML 
                    const float proj_uSr = pML->mTrackProjStartX - F.mbf/pML->mTrackStartDepth;
                    const float proj_vSr = pML->mTrackProjStartY; 

                    const float proj_uEr = pML->mTrackProjEndX - F.mbf/pML->mTrackEndDepth;
                    const float proj_vEr = pML->mTrackProjEndY;  

                    Geom2DUtils::GetLine2dRepresentation(proj_uSr, proj_vSr, proj_uEr, proj_vEr, projRightLineRepresentation);  

                    // get current frame line end-points on right image
                    const float uSr = F.mvuRightLineStart[idx];
                    const float vSr = kl.startPointY;   

                    const float uEr = F.mvuRightLineEnd[idx];
                    const float vEr = kl.endPointY;                                                       

                    // distance point-line + point-line on right line
                    // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                    // constraint: 0 = l1*u1 + l2*v1 + l3  
                    const float distSr = projRightLineRepresentation.nx*uSr + projRightLineRepresentation.ny*vSr - projRightLineRepresentation.d;
                    const float distEr = projRightLineRepresentation.nx*uEr + projRightLineRepresentation.ny*vEr - projRightLineRepresentation.d;
                    //const float err2r = distSr*distSr + distEr*distEr; 
                    const float distSr2 = distSr*distSr;
                    const float distEr2 = distEr*distEr;

                    //if(err2r*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLineMonoProj)  
                    if(distSr2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj || 
                       distEr2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj)
                    {
                        continue;
                    }                
                }  
    #endif  
                        
                const cv::Mat &d = F.mLineDescriptors.row(idx);

                const int dist = DescriptorDistance(MPdescriptor,d);

                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestLevel2 = bestLevel;
                    //bestLevel = F.mvKeyLinesUn[idx].octave;
                    bestLevel = kl.octave;
                    bestIdx=idx;
                }
                else if(dist<bestDist2)
                {
                    //bestLevel2 = F.mvKeyLinesUn[idx].octave;
                    bestLevel2 = kl.octave;
                    bestDist2=dist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if( (bestDist<=TH_HIGH) && (bestIdx>=0) )
            {
                if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                    continue;
                //if(bestDist>mfNNratio*bestDist2)
                //    continue;            

                //std::cout << "match distance H: " << bestDist << std::endl; 
                
                F.mvpMapLines[bestIdx]=pML;

                if(F.NlinesLeft != -1 && F.mvLeftToRightLinesMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                    F.mvpMapLines[F.mvLeftToRightLinesMatch[bestIdx] + F.NlinesLeft] = pML;
                    nmatches++;
                }

                nmatches++;
            }
        }

        if(F.NlinesLeft!=-1 && pML->mbTrackInViewR)
        {
            // check on right image 
            
            const int &nPredictedLevel = pML->mnTrackScaleLevelR;
            if(nPredictedLevel==-1) continue; 
    
            Geom2DUtils::GetLine2dRepresentation(pML->mTrackProjStartXR, pML->mTrackProjStartYR, pML->mTrackProjEndXR, pML->mTrackProjEndYR, projLineRepresentation); 
            
            const float scale = F.mvLineScaleFactors[nPredictedLevel];
            const float deltaTheta = Frame::kDeltaTheta*scale;
            const float deltaD = Frame::kDeltaD*scale;        
            const vector<size_t> vIndices = F.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nPredictedLevel-1,nPredictedLevel, true /*bRight*/);

            if(vIndices.empty())
                continue;

            const cv::Mat MPdescriptor = pML->GetDescriptor();

            int bestDist  = 256;
            int bestLevel= -1;
            int bestDist2 = 256;
            int bestLevel2 = -1;
            int bestIdx = -1;

            // Get best and second matches with near keylines
            for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)   
            {
                const size_t idx = *vit; // this is a right line index

                if(F.mvpMapLines[idx])
                    if(F.mvpMapLines[idx]->Observations()>0)
                        continue;

                const cv::line_descriptor_c::KeyLine& kl = F.mvKeyLinesRightUn[idx];
                
    #if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING             
                // dist segment-segment 
                const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(pML->mTrackProjStartXR, pML->mTrackProjStartYR), 
                                                                            Eigen::Vector2f(pML->mTrackProjEndXR, pML->mTrackProjEndYR),
                                                                            Eigen::Vector2f(kl.startPointX, kl.startPointY),
                                                                            Eigen::Vector2f(kl.endPointX, kl.endPointY));

                if(distSegSeg*distSegSeg*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareSegSeg) 
                {
                    continue;
                }
    #else 
                // distance point-line + point-line
                // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                // constraint: 0 = l1*u1 + l2*v1 + l3  
                const float distS = projLineRepresentation.nx*kl.startPointX + projLineRepresentation.ny*kl.startPointY - projLineRepresentation.d;
                const float distE = projLineRepresentation.nx*kl.endPointX + projLineRepresentation.ny*kl.endPointY - projLineRepresentation.d;
                //const float err2 = distS*distS + distE*distE; 
                const float distS2 = distS*distS;
                const float distE2 = distE*distE;

                //if(err2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLineMonoProj)  
                if(distS2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj || 
                   distE2*(F.mvLineInvLevelSigma2[nPredictedLevel])>thChiSquareLinePointProj)

                {
                    continue;
                }    
    #endif 
 
                const cv::Mat &d = F.mLineDescriptors.row(idx + F.NlinesLeft);

                const int dist = DescriptorDistance(MPdescriptor,d);

                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestLevel2 = bestLevel;
                    bestLevel = kl.octave;
                    bestIdx=idx;
                }
                else if(dist<bestDist2)
                {
                    bestLevel2 = kl.octave;
                    bestDist2=dist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if( (bestDist<=TH_HIGH) && (bestIdx>=0) )
            {
                if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                    continue;
                //if(bestDist>mfNNratio*bestDist2)
                //    continue;            

                //std::cout << "match distance H: " << bestDist << std::endl; 
                
                F.mvpMapLines[bestIdx + F.NlinesLeft]=pML;

                if(F.NlinesLeft != -1 && F.mvRightToLeftLinesMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                    F.mvpMapLines[F.mvRightToLeftLinesMatch[bestIdx]] = pML;
                    nmatches++;
                }

                nmatches++;
            }
        }
     
    }

#if VERBOSE    
    std::cout << "LineMatcher::MatchByProjection() F-Map - matched " << nmatches << " lines" << std::endl;
#endif
    
    return nmatches;
}


/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 

#if 0
// guided search by probjection; at present time not used!
int LineMatcher::SearchByProjection(Frame &CurrentFrame, KeyFramePtr& pKF, const set<MapLinePtr> &sAlreadyFound, const bool bLargerSearch, const int& descriptorDist)
{
    int nmatches = 0;

    //const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    //const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    //const cv::Mat Ow = -Rcw.t()*tcw;

    const Sophus::SE3f Tcw = CurrentFrame.GetPose();
    const Eigen::Matrix3f Rcw = CurrentFrame.GetRcw();
    const Eigen::Vector3f tcw = Tcw.translation();    

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(100);
    constexpr float factor = HISTO_LENGTH/M_2PI; // NOTE: kl.angle for lines is in radians and not in degrees

    const vector<MapLinePtr> vpMLs = pKF->GetMapLineMatches();

    LineProjection proj;
    const FrameTransformsForLineProjection currFrameTransforms(CurrentFrame);    
    Line2DRepresentation projLineRepresentation;
    
    for(size_t i=0, iend=vpMLs.size(); i<iend; i++)
    {
        MapLinePtr pML = vpMLs[i];

        if(pML)
        {
            if(!pML->isBad() && !sAlreadyFound.count(pML))
            {
                //Project
//                cv::Mat x3Dw = pML->GetWorldPos();
//                cv::Mat x3Dc = Rcw*x3Dw+tcw;
//
//                const float xc = x3Dc.at<float>(0);
//                const float yc = x3Dc.at<float>(1);
//                const float invzc = 1.0/x3Dc.at<float>(2);
//
//                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
//                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;
//
//                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
//                    continue;
//                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
//                    continue;
//
//                // Compute predicted scale level
//                cv::Mat PO = x3Dw-Ow;
//                float dist3D = cv::norm(PO);
//
//                const float maxDistance = pML->GetMaxDistanceInvariance();
//                const float minDistance = pML->GetMinDistanceInvariance();
//
//                // Depth must be inside the scale pyramid of the image
//                if(dist3D<minDistance || dist3D>maxDistance)
//                    continue;
//
//                int nPredictedLevel = pML->PredictScale(dist3D,&CurrentFrame);
//
//                // Search in a window
//                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];
                
                //if(!proj.ProjectLineWithCheck(Rcw, tcw, pML, CurrentFrame))
		        if(!proj.ProjectLineWithCheck(pML, CurrentFrame, currFrameTransforms))
                    continue;
                
                int nPredictedLevel = pML->PredictScale(proj.distMiddlePoint,&CurrentFrame);                
                
                Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation); 
                
                //const vector<size_t> vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation);    
                
                const float scale = CurrentFrame.mvLineScaleFactors[nPredictedLevel];
                const float deltaTheta = Frame::kDeltaTheta*scale;
                const float deltaD = Frame::kDeltaD*scale;                
                const vector<size_t> vIndices2 = CurrentFrame.GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD, nPredictedLevel-1,nPredictedLevel);                

                //const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dML = pML->GetDescriptor();

                int bestDist  = 256;
                int bestIdx   = -1;
                int bestDist2 = 256;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapLines[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mLineDescriptors.row(i2);

                    const int dist = DescriptorDistance(dML,d);

//                    if(dist<bestDist)
//                    { 
//                        bestDist=dist;
//                        bestIdx=i2;
//                    }
                    
                    if(dist<bestDist)
                    {
                        bestDist2=bestDist;
                        bestDist=dist;
                        bestIdx=i2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                    
                }

                if( (bestDist<=descriptorDist) && (bestIdx>=0) )
                {
                    
                    if(bestDist>mfNNratio*bestDist2)
                        continue;    
                                        
                    CurrentFrame.mvpMapLines[bestIdx]=pML;
                    nmatches++;

                   if(mbCheckOrientation)
                   {
                       float rot = pKF->mvKeyLinesUn[i].angle-CurrentFrame.mvKeyLinesUn[bestIdx].angle;
                       if(rot<0.0){ rot+=M_2PI; } else if(rot>M_2PI) { rot-=M_2PI; } // NOTE: kl.angle for lines is in radians and not in degrees
                       int bin = round(rot*factor);
                       if(bin==HISTO_LENGTH)
                           bin=0;
                       assert(bin>=0 && bin<HISTO_LENGTH);
                       rotHist[bin].push_back(bestIdx);
                   }
                }

            }
        }
    }

   if(mbCheckOrientation)
   {
       int ind1=-1;
       int ind2=-1;
       int ind3=-1;

       ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

       for(int i=0; i<HISTO_LENGTH; i++)
       {
           if(i!=ind1 && i!=ind2 && i!=ind3)
           {
               for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
               {
                   CurrentFrame.mvpMapLines[rotHist[i][j]]=static_cast<MapLinePtr>(NULL);
                   nmatches--;
               }
           }
       }
   }

    return nmatches;
}
#endif 


/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 


// guided search by projection; used in LoopClosing::ComputeSim3() when we use lines for voting loop closures 
// vpMatched are the lines matched in the keyframe pKF 
int LineMatcher::SearchByProjection(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const vector<MapLinePtr> &vpLines, vector<MapLinePtr> &vpMatched, int th, float ratioHamming)
{
    // Transform Scw to SE3 
    Eigen::Matrix3f Rcw = Scw.rotationMatrix();
    float scale = Scw.scale(); 
    Eigen::Vector3f tcw = Scw.translation()/scale;
    Eigen::Vector3f Ow = -Rcw.transpose()*tcw;    

    // Set of MapLines already found in the KeyFrame
    unordered_set<MapLinePtr> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapLinePtr>(NULL));

    int nmatches=0;

    LineProjection proj;
    const FrameTransformsForLineProjection frameTransforms(Rcw,tcw); 
    Line2DRepresentation projLineRepresentation;
    
    // For each Candidate MapLine Project and Match
    for(int iML=0, iendML=vpLines.size(); iML<iendML; iML++)
    {
        MapLinePtr pML = vpLines[iML];

        // Discard Bad MapLines and already found
        if(pML->isBad() || spAlreadyFound.count(pML))
            continue;

//        // Get 3D Coords.
//        cv::Mat p3Dw = pMP->GetWorldPos();
//
//        // Transform into Camera Coords.
//        cv::Mat p3Dc = Rcw*p3Dw+tcw;
//
//        // Depth must be positive
//        if(p3Dc.at<float>(2)<0.0)
//            continue;
//
//        // Project into Image
//        const float invz = 1/p3Dc.at<float>(2);
//        const float x = p3Dc.at<float>(0)*invz;
//        const float y = p3Dc.at<float>(1)*invz;
//
//        const float u = fx*x+cx;
//        const float v = fy*y+cy;
//
//        // Point must be inside the image
//        if(!pKF->IsInImage(u,v))
//            continue;
//
//        // Depth must be inside the scale invariance region of the point
//        const float maxDistance = pMP->GetMaxDistanceInvariance();
//        const float minDistance = pMP->GetMinDistanceInvariance();
//        cv::Mat PO = p3Dw-Ow;
//        const float dist = cv::norm(PO);
//
//        if(dist<minDistance || dist>maxDistance)
//            continue;

        
        //if(!proj.ProjectLineWithCheck(Rcw, tcw, pML, *pKF))
        if(!proj.ProjectLineWithCheck(pML, *pKF, frameTransforms))
            continue;
                
        Eigen::Vector3f PO = proj.p3DMw-Ow;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pML->GetNormal();

        if(PO.dot(Pn)<0.5*proj.distMiddlePoint)
            continue;
        
        Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation); 
        
        int nPredictedLevel = pML->PredictScale(proj.distMiddlePoint,pKF);
        const float scale = pKF->mvLineScaleFactors[nPredictedLevel];
        const float deltaTheta = Frame::kDeltaTheta*scale;
        const float deltaD = th * Frame::kDeltaD*scale; // increase just the distance            
        const vector<size_t> vIndices = pKF->GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD);    
                        
    //    // Search in a radius
    //    const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
    //    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keyline in the radius
        const cv::Mat dML = pML->GetDescriptor();

        int bestDist  = 256;
        int bestIdx   = -1;
        //int bestDist2 = 256;
        
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit; // this is a left line index
            if(vpMatched[idx])
                continue;

            const int &klLevel= pKF->mvKeyLinesUn[idx].octave;

            if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

            const int dist = DescriptorDistance(dML,dKF);
 
           if(dist<bestDist)
           {
               bestDist = dist;
               bestIdx = idx;
           }
            
            // if(dist<bestDist)
            // {
            //     bestDist2=bestDist;
            //     bestDist=dist;
            //     bestIdx=idx;
            // }
            // else if(dist<bestDist2)
            // {
            //     bestDist2=dist;
            // }
        }

        
        if( (bestDist<=TH_LOW*ratioHamming) && (bestIdx>=0) )            
        {
            // < N.B.: not used for points 
            //if(bestDist>mfNNratio*bestDist2)
            //    continue;    
                             
            //std::cout << "LC: match distance L: " << bestDist << std::endl; 
            
            vpMatched[bestIdx]=pML;
            nmatches++;
        }

    }

    return nmatches;
}

// Project MapLines using a Similarity Transformation and search matches.
// Used in Place Recognition (Loop Closing and Merging)
int LineMatcher::SearchByProjection(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const std::vector<MapLinePtr> &vpLines, const std::vector<KeyFramePtr> &vpLinesKFs,
                                    std::vector<MapLinePtr> &vpMatched, std::vector<KeyFramePtr> &vpMatchedKF, int th, float ratioHamming)
{
    // Transform Scw to SE3 
    Eigen::Matrix3f Rcw = Scw.rotationMatrix();
    float scale = Scw.scale(); 
    Eigen::Vector3f tcw = Scw.translation()/scale;
    Eigen::Vector3f Ow = -Rcw.transpose()*tcw;     

    // Set of MapLines already found in the KeyFrame
    unordered_set<MapLinePtr> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapLinePtr>(NULL));

    int nmatches=0;
    LineProjection proj;
    const FrameTransformsForLineProjection frameTransforms(Rcw,tcw); 
    Line2DRepresentation projLineRepresentation;
    
    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpLines.size(); iMP<iendMP; iMP++)
    {
        MapLinePtr pML = vpLines[iMP];
        KeyFramePtr pKFi = vpLinesKFs[iMP];

        // Discard Bad MapPoints and already found
        if(pML->isBad() || spAlreadyFound.count(pML))
            continue;
        
    //    // Get 3D Coords.
    //    cv::Mat p3Dw = pML->GetWorldPos();

    //    // Transform into Camera Coords.
    //    cv::Mat p3Dc = Rcw*p3Dw+tcw;

    //    // Depth must be positive
    //    if(p3Dc.at<float>(2)<0.0)
    //        continue;

    //    // Project into Image
    //    const float invz = 1/p3Dc.at<float>(2);
    //    const float x = p3Dc.at<float>(0)*invz;
    //    const float y = p3Dc.at<float>(1)*invz;

    //    const float u = fx*x+cx;
    //    const float v = fy*y+cy;

    //    // Point must be inside the image
    //    if(!pKF->IsInImage(u,v))
    //        continue;

    //    // Depth must be inside the scale invariance region of the point
    //    const float maxDistance = pML->GetMaxDistanceInvariance();
    //    const float minDistance = pML->GetMinDistanceInvariance();
    //    cv::Mat PO = p3Dw-Ow;
    //    const float dist = cv::norm(PO);

    //    if(dist<minDistance || dist>maxDistance)
    //        continue;

        //if(!proj.ProjectLineWithCheck(Rcw, tcw, pML, *pKF))
	    if(!proj.ProjectLineWithCheck(pML, *pKF, frameTransforms))
            continue;

        Eigen::Vector3f PO = proj.p3DMw-Ow;
        
        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pML->GetNormal(); 

        if(PO.dot(Pn)<0.5*proj.distMiddlePoint)
            continue;
        
        Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation); 
        
        int nPredictedLevel = pML->PredictScale(proj.distMiddlePoint,pKF);        

    //    // Search in a radius
    //    const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
    //    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);
        
        const float scale = pKF->mvLineScaleFactors[nPredictedLevel];
        const float deltaTheta = Frame::kDeltaTheta*scale;
        const float deltaD = th * Frame::kDeltaD*scale;  //only increase the distance 
        const vector<size_t> vIndices = pKF->GetLineFeaturesInArea(projLineRepresentation, deltaTheta, deltaD);            

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dML = pML->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit; // this is a lef line index
            if(vpMatched[idx])
                continue;

            const int &klLevel= pKF->mvKeyLinesUn[idx].octave;

            if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);            

            const int dist = DescriptorDistance(dML,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if( (bestDist<=TH_LOW*ratioHamming) && (bestIdx>=0) ) 
        {
            vpMatched[bestIdx] = pML;
            vpMatchedKF[bestIdx] = pKFi;
            nmatches++;
        }

    }

    return nmatches;
}

/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 

// used in LocalMapping::SearchInNeighbors()
// Project MapLines into KeyFrame and search for duplicated MapLines
int LineMatcher::Fuse(KeyFramePtr& pKF, const vector<MapLinePtr> &vpMapLines, const float th, const bool bRight) 
{
    if(bRight) MSG_ASSERT(pKF->NlinesLeft!=-1,"You can't ask for right lines without a proper left and right line management!");

    // Eigen::Matrix3f Rcw = pKF->GetRotation();
    // Eigen::Vector3f tcw = pKF->GetTranslation();
    Eigen::Vector3f Ow = bRight ? pKF->GetRightCameraCenter() : pKF->GetCameraCenter();

    int nFused=0;

    const int nMLs = vpMapLines.size();

    LineProjection proj;
    const FrameTransformsForLineProjection frameTransforms(*pKF);
    Line2DRepresentation projLineRepresentation;
    Line2DRepresentation projRightLineRepresentation;    
    
    for(int i=0; i<nMLs; i++)
    {
        MapLinePtr pML = vpMapLines[i];

        if(!pML)
            continue;

        if(pML->isBad() || pML->IsInKeyFrame(pKF))
            continue;

    //    const cv::Mat p3DSw = pML->GetWorldPosStart(); 
    //    const cv::Mat p3DEw   = pML->GetWorldPosEnd();    
    //    const cv::Mat p3DMw = 0.5*(p3DSw+p3DEw); // middle point 
    //    const cv::Mat p3DMc = Rcw*p3DMw + tcw;

    //    // Depth must be positive
    //    if(p3DMc.at<float>(2)<0.0f)
    //        continue;

    //    const float invz = 1/p3DMc.at<float>(2);
    //    const float x = p3DMc.at<float>(0)*invz;
    //    const float y = p3DMc.at<float>(1)*invz;

    //    const float u = fx*x+cx;
    //    const float v = fy*y+cy;

    //    // Point must be inside the image
    //    if(!pKF->IsInImage(u,v))
    //        continue;
        
        ///

        // project line to current frame 
        //if (!proj.ProjectLineWithCheck(Rcw, tcw, pML, *pKF))
	    if (!proj.ProjectLineWithCheck(pML, *pKF, frameTransforms, bRight))
            continue;
  
        ///

    //    const float ur = u-bf*invz;

    //    const float maxDistance = pML->GetMaxDistanceInvariance();
    //    const float minDistance = pML->GetMinDistanceInvariance();
        Eigen::Vector3f PO = proj.p3DMw-Ow;
    //    const float dist3D = cv::norm(PO);

    //     Depth must be inside the scale pyramid of the image
    //    if(dist3D<minDistance || dist3D>maxDistance )
    //        continue;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pML->GetNormal(); 

        if(PO.dot(Pn)<0.5*proj.distMiddlePoint)
            continue;

        int nPredictedLevel = pML->PredictScale(proj.distMiddlePoint,pKF);

    //    // Search in a radius
    //    const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
    //    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);
        
        Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation);
        
        const float scale = pKF->mvLineScaleFactors[nPredictedLevel];
        const float deltaTheta = Frame::kDeltaTheta*scale;     
        const float deltaD = th * Frame::kDeltaD*scale; // increase just the distance               
        const vector<size_t>& vIndices = pKF->GetLineFeaturesInArea(projLineRepresentation,deltaTheta,deltaD,bRight);  
        
        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dML = pML->GetDescriptor();

        int bestDist  = 256;
        int bestIdx   = -1;
        //int bestDist2 = 256; // second to best 
        
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            size_t idx = *vit; // this can be left or right line index depending on bRight

            //const cv::line_descriptor_c::KeyLine &kl = pKF->mvKeyLinesUn[idx];
            const cv::line_descriptor_c::KeyLine &kl = bRight? pKF->mvKeyLinesRightUn[idx] : pKF->mvKeyLinesUn[idx]; 
            const int klLevel= kl.octave;

            if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                continue;

        //    if(pKF->mvuRight[idx]>=0)
        //    {
        //        // Check reprojection error in stereo
        //        const float &kpx = kl.pt.x;
        //        const float &kpy = kl.pt.y;
        //        const float &kpr = pKF->mvuRight[idx];
        //        const float ex = u-kpx;
        //        const float ey = v-kpy;
        //        const float er = ur-kpr;
        //        const float e2 = ex*ex+ey*ey+er*er;

        //        if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8) // 7.81 = value of the inverse cumulative chi-squared with 3 DOFs (Hartley-Zisserman pag 567) 
        //            continue;
        //    }
        //    else
        //    {
        //        const float &kpx = kl.pt.x;
        //        const float &kpy = kl.pt.y;
        //        const float ex = u-kpx;
        //        const float ey = v-kpy;
        //        const float e2 = ex*ex+ey*ey;

        //        if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99) // 5.99 = value of the inverse cumulative chi-squared with 2 DOFs (Hartley-Zisserman pag 567)  
        //            continue;
        //    }

#if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING 
            // dist segment-segment 
            const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(proj.uS, proj.vS), 
                                                                        Eigen::Vector2f(proj.uE, proj.vE),
                                                                        Eigen::Vector2f(kl.startPointX, kl.startPointY),
                                                                        Eigen::Vector2f(kl.endPointX, kl.endPointY));
            
            /*const float distLine = std::max(
            fabs(projLineRepresentation.nx*kl.startPointX + projLineRepresentation.ny*kl.startPointY - projLineRepresentation.d),
            fabs(projLineRepresentation.nx*kl.endPointX + projLineRepresentation.ny*kl.endPointY - projLineRepresentation.d));*/
             
            if(distSegSeg*distSegSeg*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareSegSeg) 
            {
                //std::cout<< "distLineLine: " << distLineLine << std::endl; 
                //std::cout<< "distSegSeg: " << distSegSeg << std::endl; 
                continue;
            }            
#else 
            // distance point-line + point-line
            // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
            // constraint: 0 = l1*u1 + l2*v1 + l3  
            const float distS = projLineRepresentation.nx*kl.startPointX + projLineRepresentation.ny*kl.startPointY - projLineRepresentation.d;
            const float distE = projLineRepresentation.nx*kl.endPointX + projLineRepresentation.ny*kl.endPointY - projLineRepresentation.d;
            //const float err2 = distS*distS + distE*distE; 
            const float distS2 = distS*distS;
            const float distE2 = distE*distE;

            //if(err2*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLineMonoProj)  
            if(distS2*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLinePointProj || 
               distE2*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLinePointProj)
            {
                continue;
            } 
#endif             
            

#if USE_STEREO_PROJECTION_CHECK
            // fast stereo projection check with rectified images
            if(!pKF->mpCamera2 && (!pKF->mvuRightLineStart.empty()) && pKF->mvuRightLineStart[idx]>=0 && pKF->mvuRightLineEnd[idx]>=0)
            {
                // we assume left and right images are rectified 

                // get right projection of pML 
                const float proj_uSr = proj.uS - pKF->mbf*proj.invSz;
                const float proj_vSr = proj.vS; 

                const float proj_uEr = proj.uE - pKF->mbf*proj.invEz;
                const float proj_vEr = proj.vE;  

                Geom2DUtils::GetLine2dRepresentation(proj_uSr, proj_vSr, proj_uEr, proj_vEr, projRightLineRepresentation);  

                // get current frame line end-points on right image
                const float uSr = pKF->mvuRightLineStart[idx];
                const float vSr = kl.startPointY;   

                const float uEr = pKF->mvuRightLineEnd[idx];
                const float vEr = kl.endPointY;                                                       

                // distance point-line + point-line on right line
                // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                // constraint: 0 = l1*u1 + l2*v1 + l3  
                const float distSr = projRightLineRepresentation.nx*uSr + projRightLineRepresentation.ny*vSr - projRightLineRepresentation.d;
                const float distEr = projRightLineRepresentation.nx*uEr + projRightLineRepresentation.ny*vEr - projRightLineRepresentation.d;
                //const float err2r = distSr*distSr + distEr*distEr; 
                const float distSr2 = distSr*distSr;
                const float distEr2 = distEr*distEr;

                //if(err2r*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLineMonoProj)  
                if(distSr2*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLinePointProj || 
                   distEr2*(pKF->mvLineInvLevelSigma2[klLevel])>kChiSquareLinePointProj)
                {
                    continue;
                }                
            }  
#endif  

            if(bRight) idx += pKF->NlinesLeft;

            const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

            const int dist = DescriptorDistance(dML,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
            
            // if(dist<bestDist)
            // {
            //     bestDist2=bestDist;
            //     bestDist=dist;
            //     bestIdx=idx;
            // }
            // else if(dist<bestDist2)
            // {
            //     bestDist2=dist;
            // }
            
        }

        // If there is already a MapLine replace otherwise add new measurement
        if( (bestDist<=TH_LOW) && (bestIdx>=0) )
        {            
            // < N.B.: !not used in point fusion!
            //if(bestDist>mfNNratio*bestDist2)
            //    continue;    
            
            //std::cout << "match distance FUSE L: " << bestDist << std::endl; 
            
            MapLinePtr pMLinKF = pKF->GetMapLine(bestIdx);
            if(pMLinKF)
            {
                if(!pMLinKF->isBad())
                {
                    if(pMLinKF->Observations() > pML->Observations())
                        pML->Replace(pMLinKF);
                    else
                        pMLinKF->Replace(pML);
                }
            }
            else
            {
                //if( pML->AddObservation(pKF,bestIdx) )
                //    pKF->AddMapLine(pML,bestIdx);
                pML->AddObservation(pKF,bestIdx);
                pKF->AddMapLine(pML,bestIdx);
            }
            nFused++;
        }
    }  

#if VERBOSE    
    if(nFused>0)
        std::cout << "LineMatcher::Fuse() KF-Map - fused " << nFused << " lines" << std::endl;
#endif
        
    return nFused;
}

/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 

// used in LoopClosing::SearchAndFuse()
// Project MapLines into KeyFrame using a given Sim3 and search for duplicated MapLines
int LineMatcher::Fuse(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const std::vector<MapLinePtr> &vpLines, const float th, vector<MapLinePtr> &vpReplaceLine)
{
    // Transform Scw to SE3 
    Eigen::Matrix3f Rcw = Scw.rotationMatrix();
    float scale = Scw.scale(); 
    Eigen::Vector3f tcw = Scw.translation()/scale;
    Eigen::Vector3f Ow = -Rcw.transpose()*tcw;      

    // Set of MapLines already found in the KeyFrame
    const unordered_set<MapLinePtr> spAlreadyFound = pKF->GetMapLinesUnordered();

    int nFused=0;

    const int nLines = vpLines.size();
    
    LineProjection proj;
    const FrameTransformsForLineProjection frameTransforms(Rcw,tcw);
    Line2DRepresentation projLineRepresentation;
    Line2DRepresentation projRightLineRepresentation;    

    // For each candidate MapLine project and match
    for(int iML=0; iML<nLines; iML++)
    {
        MapLinePtr pML = vpLines[iML];

        // Discard Bad MapLines and already found
        if(pML->isBad() || spAlreadyFound.count(pML))
            continue;

//        // Get 3D Coords.
//        cv::Mat p3Dw = pMP->GetWorldPos();
//
//        // Transform into Camera Coords.
//        cv::Mat p3Dc = Rcw*p3Dw+tcw;
//
//        // Depth must be positive
//        if(p3Dc.at<float>(2)<0.0f)
//            continue;
//
//        // Project into Image
//        const float invz = 1.0/p3Dc.at<float>(2);
//        const float x = p3Dc.at<float>(0)*invz;
//        const float y = p3Dc.at<float>(1)*invz;
//
//        const float u = fx*x+cx;
//        const float v = fy*y+cy;
//
//        // Point must be inside the image
//        if(!pKF->IsInImage(u,v))
//            continue;
//
//        // Depth must be inside the scale pyramid of the image
//        const float maxDistance = pMP->GetMaxDistanceInvariance();
//        const float minDistance = pMP->GetMinDistanceInvariance();
//        cv::Mat PO = p3Dw-Ow;
//        const float dist3D = cv::norm(PO);
//
//        if(dist3D<minDistance || dist3D>maxDistance)
//            continue;
        
//        // Viewing angle must be less than 60 deg
//        cv::Mat Pn = pMP->GetNormal();
//
//        if(PO.dot(Pn)<0.5*dist3D)
//            continue;
//
//        // Compute predicted scale level
//        const int nPredictedLevel = pML->PredictScale(dist3D,pKF);
//
//        // Search in a radius
//        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];
//        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);
        

        // project line to current frame 
        //if (!proj.ProjectLineWithCheck(Rcw, tcw, pML, *pKF))
	    if (!proj.ProjectLineWithCheck(pML, *pKF, frameTransforms))
            continue;
  
        Eigen::Vector3f PO = proj.p3DMw-Ow;

        // Viewing angle must be less than 60 deg
        Eigen::Vector3f Pn = pML->GetNormal();

        if(PO.dot(Pn)<0.5*proj.distMiddlePoint)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pML->PredictScale(proj.distMiddlePoint,pKF);
        
        Geom2DUtils::GetLine2dRepresentation(proj.uS, proj.vS, proj.uE, proj.vE, projLineRepresentation);
        
        const float scale = pKF->mvLineScaleFactors[nPredictedLevel];
        const float deltaTheta = Frame::kDeltaTheta*scale;
        const float deltaD = th * Frame::kDeltaD*scale; // increase just the distance              
        const vector<size_t>& vIndices = pKF->GetLineFeaturesInArea(projLineRepresentation,deltaTheta,deltaD);  
        

        if(vIndices.empty())
            continue;

        // Match to the most similar keyline in the radius

        const cv::Mat dML = pML->GetDescriptor();

        int bestDist  = INT_MAX;
        int bestIdx   = -1;
        //int bestDist2 = INT_MAX;
        
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit; // this is a left line index
            
            const cv::line_descriptor_c::KeyLine &kl = pKF->mvKeyLinesUn[idx];
            const int &klLevel= kl.octave;
            
            if(klLevel<nPredictedLevel-1 || klLevel>nPredictedLevel)
                continue;
            
#if USE_DISTSEGMENT2SEGMENT_FOR_MATCHING // added: it is not used in sim3 search and fuse for points            
            // dist segment-segment  
            const float distSegSeg = Geom2DUtils::distSegment2Segment(Eigen::Vector2f(proj.uS, proj.vS), 
                                                                      Eigen::Vector2f(proj.uE, proj.vE),
                                                                      Eigen::Vector2f(kl.startPointX, kl.startPointY),
                                                                      Eigen::Vector2f(kl.endPointX, kl.endPointY));
            
            if(distSegSeg*distSegSeg*(pKF->mvLineInvLevelSigma2[nPredictedLevel])>kChiSquareSegSeg) 
            {
                //std::cout<< "distLineLine: " << distLineLine << std::endl; 
                continue;
            }          
#else 
            // distance point-line + point-line
            // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
            // constraint: 0 = l1*u1 + l2*v1 + l3  
            const float distS = projLineRepresentation.nx*kl.startPointX + projLineRepresentation.ny*kl.startPointY - projLineRepresentation.d;
            const float distE = projLineRepresentation.nx*kl.endPointX + projLineRepresentation.ny*kl.endPointY - projLineRepresentation.d;
            //const float err2 = distS*distS + distE*distE; 
            const float distS2 = distS*distS;
            const float distE2 = distE*distE;

            //if(err2*(pKF->mvLineInvLevelSigma2[nPredictedLevel])>kChiSquareLineMonoProj)  
            if( distS2*(pKF->mvLineInvLevelSigma2[nPredictedLevel])>kChiSquareLinePointProj || 
                distE2*(pKF->mvLineInvLevelSigma2[nPredictedLevel])>kChiSquareLinePointProj)
            {
                continue;
            } 
#endif                 


#if 0 //USE_STEREO_PROJECTION_CHECK  // disabled: it is not used in sim3 search and fuse for points
            if(!pKF->mpCamera2 && (!pKF->mvuRightLineStart.empty()) && pKF->mvuRightLineStart[idx]>=0 && pKF->mvuRightLineEnd[idx]>=0)
            {
                // we assume left and right images are rectified 

                // get right projection of pML 
                const float proj_uSr = proj.uS - pKF->mbf*proj.invSz;
                const float proj_vSr = proj.vS; 

                const float proj_uEr = proj.uE - pKF->mbf*proj.invEz;
                const float proj_vEr = proj.vE;  

                Geom2DUtils::GetLine2dRepresentation(proj_uSr, proj_vSr, proj_uEr, proj_vEr, projRightLineRepresentation);  

                // get current frame line end-points on right image
                const float uSr = pKF->mvuRightLineStart[idx];
                const float vSr = kl.startPointY;   

                const float uEr = pKF->mvuRightLineEnd[idx];
                const float vEr = kl.endPointY;                                                       

                // distance point-line + point-line on right line
                // line representation [l1,l2,l3]=[nx,ny,-d] with [nx,ny] defining a unit normal
                // constraint: 0 = l1*u1 + l2*v1 + l3  
                const float distSr = projRightLineRepresentation.nx*uSr + projRightLineRepresentation.ny*vSr - projRightLineRepresentation.d;
                const float distEr = projRightLineRepresentation.nx*uEr + projRightLineRepresentation.ny*vEr - projRightLineRepresentation.d;
                const float err2r = distSr*distSr + distEr*distEr; 

                if(err2r*(pKF->mvLineInvLevelSigma2[nPredictedLevel])>kChiSquareLineMonoProj)  
                {
                    continue;
                }                
            }  
#endif  

            const cv::Mat &dKF = pKF->mLineDescriptors.row(idx);

            int dist = DescriptorDistance(dML,dKF);

           if(dist<bestDist)
           {
               bestDist = dist;
               bestIdx = idx;
           }
            
            // if(dist<bestDist)
            // {
            //     bestDist2=bestDist;
            //     bestDist=dist;
            //     bestIdx=idx;
            // }
            // else if(dist<bestDist2)
            // {
            //     bestDist2=dist;
            // }
            
        }

        // If there is already a MapLine replace otherwise add new measurement
        if( (bestDist<=TH_LOW)  && (bestIdx>=0) )
        {
            // < N.B.: !not used in point fusion!
            //if(bestDist>mfNNratio*bestDist2)
            //    continue;    
                                         
            //std::cout << "match distance FUSE L: " << bestDist << std::endl; 
                        
            MapLinePtr pMLinKF = pKF->GetMapLine(bestIdx);
            if(pMLinKF)
            {
                if(!pMLinKF->isBad())
                    vpReplaceLine[iML] = pMLinKF;  
            }
            else
            {
                //if( pML->AddObservation(pKF,bestIdx) )
                //    pKF->AddMapLine(pML,bestIdx);
                pML->AddObservation(pKF,bestIdx);
                pKF->AddMapLine(pML,bestIdx);
            }
            nFused++;
        }
    }

#if VERBOSE    
    if(nFused>0)
        std::cout << "LineMatcher::Fuse() KF-Map - fused " << nFused << " lines for loop closure" << std::endl;
#endif
    
    return nFused;
}



/// < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < < 


int LineMatcher::ComputeDescriptorMatches(const cv::Mat& ldesc_q/*query*/, const cv::Mat& ldesc_t/*train*/, const cv::Mat& queryMask, std::vector<std::vector<cv::DMatch> >& lmatches, std::vector<bool>& vValidMatch)
{
    //mpBfm->knnMatch(ldesc_q/*query*/, ldesc_t/*train*/, lmatches, 2, queryMask, true/*compact result*/);
    
    mBdm->knnMatch(ldesc_q/*query*/, ldesc_t/*train*/, lmatches, 2 /*k*/, queryMask, true/*compact result*/);
    
    vValidMatch = std::vector<bool>(lmatches.size(),false);
            
    // sort matches by the distance between the best and second best matches
#if 0
    double sigma_dist_th = 0, sigma12_dist_th = 0;
#if USE_MAD12
    LineDescriptorMAD12(lmatches, sigma12_dist_th);
    sigma12_dist_th = sigma12_dist_th * kDescTh;
#else
    LineDescriptorMAD(lmatches, sigma_dist_th);
    sigma_dist_th = 3 * sigma_dist_th; // 3 sigma 
#endif
#endif
    
    // resort according to the queryIdx
    //sort(lmatches.begin(), lmatches.end(), sort_descriptor_by_queryIdx());
    
    int numValidMatches = 0; 

    for (size_t i=0,iEnd=lmatches.size(); i<iEnd; i++)
    {
        // check the minimum distance of matches 
#if 0
#if USE_MAD12      
        const double dist_12 = lmatches[i][1].distance - lmatches[i][0].distance;
        if (dist_12 > sigma12_dist_th) continue 
#else
        if (lmatches[i][0].distance > sigma_dist_th) continue 
#endif
#endif
        
        const std::vector<cv::DMatch>& lmatchesi = lmatches[i];
        if(lmatchesi.size() > 1)
        {
            if (lmatchesi[0].distance < mfNNratio* lmatchesi[1].distance)
            {
                vValidMatch[i] = true; 
                numValidMatches++;
            }
        }
        else
        {
            vValidMatch[i] = true; 
            numValidMatches++;
        }
    }

    return numValidMatches;    
}

void LineMatcher::LineDescriptorMAD(const std::vector<std::vector<cv::DMatch> >&matches, double &sigma_mad)
{
    std::vector<std::vector<cv::DMatch> > matches_nn;
    matches_nn = matches;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_dist_median = matches_nn[int(matches_nn.size() / 2)][0].distance;
    for (size_t j = 0, jEnd=matches_nn.size(); j < jEnd; j++)
        matches_nn[j][0].distance = fabsf(matches_nn[j][0].distance - nn_dist_median);
    sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    sigma_mad = 1.4826 * matches_nn[int(matches_nn.size() / 2)][0].distance;
}

void LineMatcher::LineDescriptorMAD12(const std::vector<std::vector<cv::DMatch> >&matches, double &sigma12_mad)
{
    std::vector<std::vector<cv::DMatch> > matches_12;
    matches_12 = matches;

    // estimate the NN's 12 distance standard deviation
    double nn12_dist_median;
    sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN12_dist());
    nn12_dist_median = matches_12[int(matches_12.size() / 2)][1].distance - matches_12[int(matches_12.size() / 2)][0].distance;
    for (size_t j = 0,jEnd=matches_12.size(); j < jEnd; j++)
        matches_12[j][0].distance = fabsf(matches_12[j][1].distance - matches_12[j][0].distance - nn12_dist_median);
    sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist());
    sigma12_mad = 1.4826 * matches_12[int(matches_12.size() / 2)][0].distance;
}

#if 0 

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int LineMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const uint32_t *pa = a.ptr<uint32_t>();
        const uint32_t *pb = b.ptr<uint32_t>();

        int dist=0;      

        for(int i=0; i<8; i++, pa++, pb++)
        {
//#include <immintrin.h>             
#ifdef __POPCNT__    
            // SIMD optimization by using 32-size instruction
            dist += _mm_popcnt_u32(*pa ^ *pb); // count number of 1 over XOR
#else                  
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
#endif             
        }
        return dist;
    }

#else 

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int LineMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const uint64_t *pa = a.ptr<uint64_t>();
        const uint64_t *pb = b.ptr<uint64_t>();

        uint64_t dist=0;      

        for(int i=0; i<4; i++, pa++, pb++)
        {
//#include <immintrin.h>             
#ifdef __POPCNT__    
            // SIMD optimization by using 64-size instruction
            dist += _mm_popcnt_u64(*pa ^ *pb); // count number of 1 over XOR
#else             
            using T = uint64_t;   
            uint64_t v = *pa ^ *pb;
            // v = v - ((v >> 1) & 0x55555555);
            // v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            // dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;

            v = v - ((v >> 1) & (T)~(T)0/3);                           // temp
            v = (v & (T)~(T)0/15*3) + ((v >> 2) & (T)~(T)0/15*3);      // temp
            v = (v + (v >> 4)) & (T)~(T)0/255*15;                      // temp
            dist += (T)(v * ((T)~(T)0/255)) >> (sizeof(T) - 1) * CHAR_BIT; // count            
#endif             
        }
        return dist;
    }    
    
#endif 

} //namespace PLVS2

