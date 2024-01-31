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

#ifndef LINE_MATCHER_H
#define LINE_MATCHER_H

#include <vector>
#include <list>

#include <opencv2/opencv.hpp>
#include"sophus/sim3.hpp"

#include "Frame.h"
#include "KeyFrame.h"
#include "MapLine.h"
#include "LineProjection.h"
#include "Pointers.h"

namespace cv
{
namespace line_descriptor_c
{
class BinaryDescriptorMatcher;
}
}


namespace PLVS2
{

   
class LineMatcher
{
    static const float kDescTh;    // parameter to avoid outliers in line matching

public: 
    
    static const int TH_HIGH;
    static const int TH_LOW;
    static const int TH_LOW_STEREO;
    static const int HISTO_LENGTH;    

public:
    
    LineMatcher(float nnratio=0.6, bool crossCheck = false, bool checkOrientation=true);

    ~LineMatcher(){}
    
    // used by Tracking::TrackReferenceKeyFrame()
    int SearchByKnn(KeyFramePtr& pKF, const Frame &F, std::vector<MapLinePtr> &vpMapLineMatches);
    
    // used by Tracking::TrackWithMotionModel()
    int SearchByKnn(Frame &CurrentFrame, const Frame &LastFrame); 
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const bool bLargerSearch, const bool bMono); 
    
    int SearchStereoMatchesByKnn(Frame &frame, std::vector<cv::DMatch>& vMatches, std::vector<bool>& vValidMatches, const int& descriptorDist);
    int SearchForTriangulation(KeyFramePtr& pKF1, KeyFramePtr& pKF2, std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);    

    // used by Tracking::SearchLocalLines()
    // Search matches between Frame keylines and projected MapLines. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByKnn(Frame &F, const std::vector<MapLinePtr> &vpMapLines);
    int SearchByProjection(Frame &F, const std::vector<MapLinePtr> &vpMapLines, const bool bLargerSearch=false);
    
    // Project MapLines seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking) [not used at the moment]
    int SearchByProjection(Frame &CurrentFrame, KeyFramePtr& pKF, const std::set<MapLinePtr> &sAlreadyFound, const bool bLargerSearch, const int& descriptorDist);    
    
    // Project MapLines using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const std::vector<MapLinePtr> &vpLines, std::vector<MapLinePtr> &vpMatched, int th, float ratioHamming);
    
    // Project MapLines using a Similarity Transformation and search matches.
    // Used in Place Recognition (Loop Closing and Merging)     
    int SearchByProjection(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const std::vector<MapLinePtr> &vpLines, const std::vector<KeyFramePtr> &vpLinesKFs,
                           std::vector<MapLinePtr> &vpMatched, std::vector<KeyFramePtr> &vpMatchedKF, int th, float ratioHamming);

    // Project MapLines into KeyFrame and search for duplicated MapLines
    int Fuse(KeyFramePtr& pKF, const std::vector<MapLinePtr> &vpMapLines, const float th=3.0, const bool bRight=false);  
    
    // Project MapLines into KeyFrame using a given Sim3 and search for duplicated MapLines
    int Fuse(KeyFramePtr& pKF, const Sophus::Sim3f& Scw, const std::vector<MapLinePtr> &vpLines, const float th, std::vector<MapLinePtr> &vpReplaceLine);
    
public: 
    
    void SetAcceptanceRatio(float nnratio) { mfNNratio = nnratio; } 
    
    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

protected:
  
    int ComputeDescriptorMatches(const cv::Mat& ldesc_q/*query*/, const cv::Mat& ldesc_t/*train*/, const cv::Mat& queryMask, std::vector<std::vector<cv::DMatch> >& lmatches_12, std::vector<bool>& vValidMatch);

    void LineDescriptorMAD(const std::vector<std::vector<cv::DMatch> >& matches, double &sigma_mad);
    void LineDescriptorMAD12(const std::vector<std::vector<cv::DMatch> >&matches, double &sigma12_mad);
        
protected:    
    
    //std::shared_ptr<cv::BFMatcher> mpBfm;
    cv::Ptr<cv::line_descriptor_c::BinaryDescriptorMatcher> mBdm; 
    
    float mfNNratio;
    bool mbCheckOrientation;    
};


} //namespace PLVS2

#endif
