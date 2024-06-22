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

#ifndef POINTCLOUDKEYFRAME_H
#define POINTCLOUDKEYFRAME_H

#include <opencv2/core/core.hpp>

#include <mutex>
#include <thread>

#include "PointDefinitions.h"

#include <boost/core/noncopyable.hpp>

#include "PointDefinitions.h"
#include "LabelMap.h"
#include "Pointers.h"

#include "Thirdparty/Sophus/sophus/se3.hpp"

#ifdef USE_CUDA
#include <opencv2/core/utility.hpp>
#include "opencv2/cudastereo.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "opencv2/calib3d.hpp"
#endif

#ifdef USE_LIBELAS   
namespace libelas
{
    class ElasInterface;
}
#endif

#ifdef USE_LIBSGM
namespace sgm
{
    class StereoSGM;
}
#endif

namespace PLVS2
{

class KeyFrame;
class StereoDisparity;

///	\class PointCloudCamParams
///	\author Luigi Freda
///	\brief Abstract class for managing camera parameters  
///	\note
///	\date
///	\warning
struct PointCloudCamParams
{
    double fx;
    double fy;
    double cx;
    double cy;
    
    double bf;

    int width;
    int height;
    
    double minDist;
    double maxDist; 

    cv::Mat mDistCoef;
    cv::Mat mK; 
};


///	\class PointCloudKeyframe
///	\author Luigi Freda
///	\brief Class for storing point clouds and associated data 
///	\note
///	\date
///	\warning
template<typename PointT>
class PointCloudKeyFrame: private boost::noncopyable
{
public:

    typedef typename pcl::PointCloud<PointT> PointCloudT;    
    typedef typename std::shared_ptr<PointCloudKeyFrame<PointT> > Ptr;
    typedef typename std::shared_ptr<const PointCloudKeyFrame<PointT> > ConstPtr;
    
public:
    
    enum StereoLibrary {kLibelas=0, kLibsgm, kLibOpenCV, kLibOpenCVCuda };
    
    static StereoLibrary skStereoLibrary; 
    static bool skbNeedRectification; 
    
public: 
        
    PointCloudKeyFrame();
    
    PointCloudKeyFrame(KeyFramePtr& pKF_in, cv::Mat& imgColor_in, cv::Mat& imgDepth_in);
    PointCloudKeyFrame(KeyFramePtr& pKF_in, cv::Mat& imgColor_in, cv::Mat& imgLeft_in, cv::Mat& imgRight_in);  
    
    ~PointCloudKeyFrame();

    void Init(PointCloudCamParams* camParams=nullptr); 
    void PreProcess(); 
    
    // set point colored point cloud w.r.t. camera frame and release color and depth   
    void SetCloud(typename PointCloudT::Ptr p_cloud_camera_in);
    
    void ReleaseColor();
    void ReleaseDepth();
    void ReleasePointIndexMat();
    void ReleaseLabelMap(); 
    void Release();

    // get Ow
    Eigen::Vector3f GetCameraCenter();
    // get Twc
    Sophus::SE3f GetCameraPose();
    
    boost::uint64_t GetTimestamp();
    
    void GetTransformedCloud(const cv::Mat& Twc, typename PointCloudT::Ptr pCloudOut);    
    
    // clear content and invalidate 
    void Clear();
    
private: 

    // libels based
    void ProcessStereoLibelas(); 
    // libsgm based 
    void ProcessStereoLibsgm();     
    
    // Opencv based 
    void ProcessStereo(); 
    // Opencv Cuda based 
    void ProcessStereoCuda();     

public: 
    
    KeyFramePtr pKF;
    cv::Mat imgColor;
    cv::Mat imgDepth;
    cv::Mat imgLeft, imgRight; // for stereo 
    cv::Mat imgPointIndex; // correspondence from pixel to  point index in cloud
    
    LabelMap labelMap; 
    
    typename PointCloudT::Ptr pCloudCamera; // cloud w.r.t. camera frame 
    
    Sophus::SE3f TwcIntegration; // pose at cloud integration time 

    bool bIsInitialized = false; 
    bool bIsProcessed = false; 
    
    bool bCloudReady;
    bool bInMap;
    bool bIsValid;
    bool bStereo;

    PointCloudCamParams* pCamParams = nullptr;
   
#ifdef USE_LIBELAS    
    static std::shared_ptr<libelas::ElasInterface> pElas;
#endif
    
#ifdef USE_LIBSGM
    static std::shared_ptr<sgm::StereoSGM> pSgm;
#endif    
    
    static std::shared_ptr<StereoDisparity> pSd;  // for OpenCV (with or without CUDA)  
        
    std::mutex keyframeMutex;
};

#if !USE_NORMALS

/// < list here the types you want to use 
template class PointCloudKeyFrame<pcl::PointXYZRGBA>;

#else

template class PointCloudKeyFrame<pcl::PointXYZRGBNormal>;
template class PointCloudKeyFrame<pcl::PointSurfelSegment>;


#endif


} //namespace PLVS2

#endif /* POINTCLOUDKEYFRAME_H */

