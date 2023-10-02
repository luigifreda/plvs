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

#ifndef LINE_EXTRACTOR_H
#define LINE_EXTRACTOR_H

#include <vector>
#include <list>

#include <opencv2/opencv.hpp>

#include <line_descriptor_custom.hpp>


namespace PLVS2
{

class LineExtractor
{
    
public:
    static bool skUseLsdExtractor; 
    
    static const int kBorderThreshold; 
    static const int kNumOctavesForBdDefault; 
    static const int kGaussianFilterSize;
    static const float kScaleFactorDefault; 
    static float skSigma0; 
    
public:
    
    LineExtractor(int numLinefeatures, cv::line_descriptor_c::LSDDetectorC::LSDOptions& opts_in);

    ~LineExtractor(){}

    void SetOptions(cv::line_descriptor_c::LSDDetectorC::LSDOptions& opts_in);
    
    // Compute the Line features and descriptors on an image.
    void operator()(const cv::Mat& image, std::vector<cv::line_descriptor_c::KeyLine>& keylines, cv::Mat& descriptors);
    
public: /// < getters 
    
    int inline GetLevels(){
        return mnLevels;}

    float inline GetScaleFactor(){
        return mfScaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
    
public: /// < setters
    
    void SetGaussianPyramid(const std::vector<cv::Mat>& pyramid, int numOctaves, float scale, int borderX_in = 0, int borderY_in = 0); 
    //void SetBorders(int borderX_in,int borderY_in);

protected:

    void detectLineFeatures(const cv::Mat& img, std::vector<cv::line_descriptor_c::KeyLine> &lines, cv::Mat &descriptors, double min_line_length);
    
protected:    
    int mnLinefeatures;

    cv::line_descriptor_c::LSDDetectorC::LSDOptions mOpts;
    cv::Ptr<cv::line_descriptor_c::BinaryDescriptor> mLbd;
    cv::Ptr<cv::line_descriptor_c::LSDDetectorC> mLsd;
    
    double mdSigmaScale;
    
    int mnLevels; 
    double mfScaleFactor;
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace PLVS2

#endif

