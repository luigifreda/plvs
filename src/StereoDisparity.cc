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


#define VERBOSE 1

#include "StereoDisparity.h"

namespace PLVS2
{

static cv::Rect computeROI(cv::Size2i src_sz, cv::Ptr<cv::StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size / 2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    cv::Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

// 


// parameters used for CPU
const double StereoParameters::kResizeScale = 0.25;
const int StereoParameters::kWinSizeSBGM = 1; //  window_size value: it should be positive and odd
const int StereoParameters::kMaxXGrad = 25;
const double StereoParameters::kSmoothingFactor = 4;
const double StereoParameters::kLambda = 8000.0;
const double StereoParameters::kSigma = 1.5;
const int StereoParameters::kMaxDisparity = 16 * 10; // should be positive and divisible by 16

StereoDisparityCPU::StereoDisparityCPU()
{
}

void StereoDisparityCPU::Init(const StereoParameters& params)
{
    mParams = params; 

    if (mParams.bDownScale)
    {
        mParams.nMaxDisparity *= mParams.dResizeScale;
        if (mParams.nMaxDisparity % 16 != 0) mParams.nMaxDisparity += 16 - (mParams.nMaxDisparity % 16);
    }
        
    // check parameters

    if (mParams.nMaxDisparity <= 0 || mParams.nMaxDisparity % 16 != 0)
    {
        std::cout << "Incorrect kMaxDisparity value: it should be positive and divisible by 16" << std::endl;
        quick_exit(-1);
    }

    if (mParams.nWinSizeSBGM <= 0 || mParams.nWinSizeSBGM % 2 != 1)
    {
        std::cout << "Incorrect window_size value: it should be positive and odd" << std::endl;
        quick_exit(-1);
    }
    
    if (mParams.nFilterType == kWlsConf)
    {    
        left_matcher = cv::StereoSGBM::create(0, mParams.nMaxDisparity, mParams.nWinSizeSBGM);
        left_matcher->setP1(24 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
        left_matcher->setP2(96 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
        left_matcher->setPreFilterCap(mParams.nMaxXGrad);
        left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);  
        
        wls_filter->setLambda(mParams.dLambda);
        wls_filter->setSigmaColor(mParams.dSigma);        
    }
    
    if (mParams.nFilterType == kWlsNoConf)
    {
        matcher = cv::StereoSGBM::create(0, mParams.nMaxDisparity, mParams.nWinSizeSBGM);
        matcher->setUniquenessRatio(0);
        matcher->setDisp12MaxDiff(1000000);
        matcher->setSpeckleWindowSize(0);
        matcher->setP1(24 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
        matcher->setP2(96 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
        matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        
        wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
        wls_filter->setDepthDiscontinuityRadius((int) ceil(0.5 * mParams.nWinSizeSBGM));    
        
        wls_filter->setLambda(mParams.dLambda);
        wls_filter->setSigmaColor(mParams.dSigma);        
    }
    
}

void StereoDisparityCPU::Compute(cv::Mat& left, cv::Mat& right, cv::Mat& disparity)
{
    cv::Mat left_for_matcher, right_for_matcher;
    cv::Mat left_disp, right_disp;
    cv::Mat filtered_disp;
    cv::Mat conf_map = cv::Mat(left.rows, left.cols, CV_8U);
    conf_map = cv::Scalar(255);
    cv::Rect ROI;
    //cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;

    if (mParams.bDownScale)
    {
        //mParams.nMaxDisparity *= mParams.dResizeScale;
        //if (mParams.nMaxDisparity % 16 != 0) mParams.nMaxDisparity += 16 - (mParams.nMaxDisparity % 16);
        cv::resize(left, left_for_matcher, cv::Size(), mParams.dResizeScale, mParams.dResizeScale);
        cv::resize(right, right_for_matcher, cv::Size(), mParams.dResizeScale, mParams.dResizeScale);
    }
    else
    {
        left_for_matcher = left.clone();
        right_for_matcher = right.clone();
    }

    if (mParams.nFilterType == kWlsConf)
    {
//        cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(0, mParams.nMaxDisparity, kWinSizeSBGM);
//        left_matcher->setP1(24 * kWinSizeSBGM * kWinSizeSBGM * kSmoothingFactor);
//        left_matcher->setP2(96 * kWinSizeSBGM * kWinSizeSBGM * kSmoothingFactor);
//        left_matcher->setPreFilterCap(kMaxXGrad);
//        left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
//        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
//        cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        matching_time = (double) cv::getTickCount();
        left_matcher-> compute(left_for_matcher, right_for_matcher, left_disp);
        right_matcher->compute(right_for_matcher, left_for_matcher, right_disp);
        matching_time = ((double) cv::getTickCount() - matching_time) / cv::getTickFrequency();


//        wls_filter->setLambda(mParams.dLambda);
//        wls_filter->setSigmaColor(mParams.dSigma);
        filtering_time = (double) cv::getTickCount();
        wls_filter->filter(left_disp, left, filtered_disp, right_disp);
        filtering_time = ((double) cv::getTickCount() - filtering_time) / cv::getTickFrequency();

        conf_map = wls_filter->getConfidenceMap();
    }
    else if (mParams.nFilterType == kWlsNoConf)
    {
//        cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(0, mParams.nMaxDisparity, mParams.nWinSizeSBGM);
//        matcher->setUniquenessRatio(0);
//        matcher->setDisp12MaxDiff(1000000);
//        matcher->setSpeckleWindowSize(0);
//        matcher->setP1(24 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
//        matcher->setP2(96 * mParams.nWinSizeSBGM * mParams.nWinSizeSBGM * mParams.dSmoothingFactor);
//        matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        ROI = computeROI(left_for_matcher.size(), matcher);
//        wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
//        wls_filter->setDepthDiscontinuityRadius((int) ceil(0.5 * mParams.nWinSizeSBGM));

        matching_time = (double) cv::getTickCount();
        matcher->compute(left_for_matcher, right_for_matcher, left_disp);
        matching_time = ((double) cv::getTickCount() - matching_time) / cv::getTickFrequency();

//        wls_filter->setLambda(kLambda);
//        wls_filter->setSigmaColor(kSigma);
        filtering_time = (double) cv::getTickCount();
        wls_filter->filter(left_disp, left, filtered_disp, cv::Mat(), ROI);
        filtering_time = ((double) cv::getTickCount() - filtering_time) / cv::getTickFrequency();
    }
    else
    {
        std::cerr << "StereoDisparityCPU::Compute() - Unsupported filter";
        quick_exit(-1);
    }

    if (mParams.bDownScale)
    {
        // upscale raw disparity and ROI back for a proper comparison:
        cv::resize(left_disp, left_disp, cv::Size(), 1 / mParams.dResizeScale, 1 / mParams.dResizeScale);
        left_disp = left_disp / mParams.dResizeScale;
    }

    disparity = filtered_disp;

#if VERBOSE    
    std::cout.precision(2);
    std::cout << "Matching time:  " << matching_time << "s" << std::endl;
    std::cout << "Filtering time: " << filtering_time << "s" << std::endl;
    std::cout << std::endl;
#endif 

}


//

StereoDisparityGPU::StereoDisparityGPU()
{
}

void StereoDisparityGPU::Init(const StereoParameters& params)
{
#ifdef USE_CUDA     

    mParams = params;
    
    // Set common parameters
    switch (mParams.nCudaAlgorithmType)
    {
    case kCudaBM:
        bm = cv::cuda::createStereoBM(mParams.nMaxDisparity);
        break;

    case kCudaBP:
        bp = cv::cuda::createStereoBeliefPropagation(mParams.nMaxDisparity);
        break;

    case kCudaCSBP:
    default:
        csbp = cv::cuda::createStereoConstantSpaceBP(mParams.nMaxDisparity);
        break;
    }
    
    pDfilter = cv::cuda::createDisparityBilateralFilter(mParams.nMaxDisparity); 

#endif
}

void StereoDisparityGPU::Compute(cv::Mat& left, cv::Mat& right, cv::Mat& disparity)
{
#ifdef USE_CUDA  

    cv::cuda::GpuMat d_left, d_right;

    if (left.empty()) throw std::runtime_error("left image empty");
    if (right.empty()) throw std::runtime_error("right image empty");
    
    if (left.channels() > 1)
    {
        std::cout << "converting left image to BW" << std::endl;
        cv::cvtColor(left, left, cv::COLOR_BGR2GRAY);
    }
    if (right.channels() > 1)
    {
        std::cout << "converting left image to BW" << std::endl;
        cv::cvtColor(right, right, cv::COLOR_BGR2GRAY);
    }

    d_left.upload(left);
    d_right.upload(right);

    // Prepare disparity map of specified type
    cv::Mat disp(left.size(), CV_8U);
    cv::cuda::GpuMat d_disp(left.size(), CV_8U);


    switch (mParams.nCudaAlgorithmType)
    {
    case StereoCudaAlgorithms::kCudaBM:
        bm->compute(d_left, d_right, d_disp);
        break;
        
    case StereoCudaAlgorithms::kCudaBP: 
        bp->compute(d_left, d_right, d_disp);
        break;
        
    case StereoCudaAlgorithms::kCudaCSBP: 
        csbp->compute(d_left, d_right, d_disp);
        break;
    }
    
    if(mParams.bCudaDisparityFilter)
    {
        /// < TODO: this filter needs tuning!!
        cv::cuda::GpuMat d_disp_filtered;
        pDfilter->apply(d_disp, d_left, d_disp_filtered);    
        d_disp_filtered.download(disp);
    }
    else
    {
        d_disp.download(disp);    
    }


    disparity = disp;

#endif
}

}