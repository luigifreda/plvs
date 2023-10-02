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

#ifndef STEREO_DISPARITY_H
#define STEREO_DISPARITY_H

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

#ifdef USE_CUDA  
#include "opencv2/cudastereo.hpp"
#endif

#include <iostream>
#include <string>

namespace PLVS2
{

enum StereoFilterTypes { kWlsConf = 0, kWlsNoConf};
static const std::string StereoFilterStrings[] = {"kWlsConf", "kWlsNoConf"};

enum StereoCudaAlgorithms { kCudaBM = 0, kCudaBP, kCudaCSBP };
static const std::string StereoCudaAlgorithmsStrings[] = {"CudaBM", "CudaBP", "CudaCSBP"};   
    
//

class StereoParameters
{
public:
    
    // default values 
    static const int kMaxDisparity;       
    static const double kResizeScale;
    static const int kWinSizeSBGM;
    static const int kMaxXGrad;
    static const double kSmoothingFactor;
    static const double kLambda;
    static const double kSigma;
 
    
public:    
    
    int nMaxDisparity = kMaxDisparity; 
    bool bDownScale = false; 
    
    double dResizeScale = kResizeScale;
    int nWinSizeSBGM = kWinSizeSBGM;
    int nMaxXGrad = kMaxXGrad;
    double dSmoothingFactor = kSmoothingFactor;
    double dLambda = kLambda;
    double dSigma = kSigma;    
    
    int nFilterType = StereoFilterTypes::kWlsConf;
    
    int nCudaAlgorithmType = StereoCudaAlgorithms::kCudaCSBP;
    bool bCudaDisparityFilter = false;
};

//

class StereoDisparity
{    
public:     
    StereoDisparity(){}
    
    virtual void Init(const StereoParameters& params) = 0;
    virtual void Compute(cv::Mat& left, cv::Mat& right, cv::Mat& disparity) = 0;
    
protected:
        
    StereoParameters mParams;
};


//

class StereoDisparityCPU: public StereoDisparity
{
public:
    
    StereoDisparityCPU();
    
    void Init(const StereoParameters& params);
    
    void Compute(cv::Mat& left, cv::Mat& right, cv::Mat& disparity);
    
protected:

    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;    
    cv::Ptr<cv::StereoSGBM> left_matcher;
    cv::Ptr<cv::StereoMatcher> right_matcher;    
    
    cv::Ptr<cv::StereoSGBM> matcher;
};

//

class StereoDisparityGPU: public StereoDisparity
{
public:
    
    StereoDisparityGPU();
    
    void Init(const StereoParameters& params);
    
    void Compute(cv::Mat& left, cv::Mat& right, cv::Mat& disparity);

protected:
    
    StereoCudaAlgorithms mAlgorithmType;
    
#ifdef USE_CUDA  
    cv::Ptr<cv::cuda::StereoBM> bm;
    cv::Ptr<cv::cuda::StereoBeliefPropagation> bp;
    cv::Ptr<cv::cuda::StereoConstantSpaceBP> csbp;   
    
    cv::Ptr<cv::cuda::DisparityBilateralFilter> pDfilter;   
#endif    
        
};

}

#endif 
