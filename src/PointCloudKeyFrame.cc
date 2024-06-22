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

#include "PointCloudKeyFrame.h"
#include "PointCloudMapping.h"

#include <pcl/common/geometry.h>

#include "KeyFrame.h" 
#include "TimeUtils.h"
#include "Converter.h"
#include "Stopwatch.h"
#include "StereoDisparity.h"
#include "Utils.h"
#include "PointCloudUtils.h"

#ifdef USE_LIBELAS  
#include "ElasInterface.h"
#endif

#ifdef USE_LIBSGM
#include "libsgm.h"
#endif

#if RERUN_ENABLED
#include "RerunSingleton.h" 
#endif 


namespace PLVS2
{

template<typename PointT>
bool PointCloudKeyFrame<PointT>::skbNeedRectification = false;

// default initialization 
template<typename PointT>
typename PointCloudKeyFrame<PointT>::StereoLibrary PointCloudKeyFrame<PointT>::skStereoLibrary = 
#ifdef USE_LIBELAS 
        PointCloudKeyFrame<PointT>::StereoLibrary::kLibelas;
#else
#ifdef USE_LIBSGM  // ---
        PointCloudKeyFrame<PointT>::StereoLibrary::kLibsgm;
#else              // ---
#ifdef USE_CUDA        
        PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCVCuda;
#else
        PointCloudKeyFrame<PointT>::StereoLibrary::kLibOpenCV;
#endif         
#endif //USE_LIBSGM              
#endif //USE_LIBELAS

#ifdef USE_LIBELAS  
template<typename PointT>
std::shared_ptr<libelas::ElasInterface> PointCloudKeyFrame<PointT>::pElas = 0;
#endif

#ifdef USE_LIBSGM
template<typename PointT>
std::shared_ptr<sgm::StereoSGM> PointCloudKeyFrame<PointT>::pSgm = 0;
#endif  

template<typename PointT>
std::shared_ptr<StereoDisparity> PointCloudKeyFrame<PointT>::pSd = 0;

template<typename PointT>
PointCloudKeyFrame<PointT>::PointCloudKeyFrame()
: pKF(nullptr), bCloudReady(false), bInMap(false), bIsValid(true), bStereo(false)
{
}

template<typename PointT>
PointCloudKeyFrame<PointT>::PointCloudKeyFrame(KeyFramePtr& pKF_in, cv::Mat& imgColor_in, cv::Mat& imgDepth_in)
: pKF(pKF_in), imgColor(imgColor_in), imgDepth(imgDepth_in), bCloudReady(false), bInMap(false), bIsValid(true), bStereo(false)
{
}

template<typename PointT>
PointCloudKeyFrame<PointT>::PointCloudKeyFrame(KeyFramePtr& pKF_in, cv::Mat& imgColor_in, cv::Mat& imgLeft_in, cv::Mat& imgRight_in)
: pKF(pKF_in), imgColor(imgColor_in), imgLeft(imgLeft_in), imgRight(imgRight_in), bCloudReady(false), bInMap(false), bIsValid(true), bStereo(true)
{
}

template<typename PointT>
PointCloudKeyFrame<PointT>::~PointCloudKeyFrame()
{
    //Clear();
}

static void convertCloneGrayImagetoCV8U(cv::Mat& img)
{
    if(img.type()!=CV_8U)
    {
        cv::Mat temp;
        img.convertTo(temp,CV_8U);      
        img = temp;
    }
    else
    {
        img = img.clone();   
    } 
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::Init(PointCloudCamParams* camParams)
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
 
    if(bIsInitialized) return; 

    if(camParams) pCamParams = camParams;
    
    if( !imgColor.empty() ) 
    {
        imgColor = imgColor.clone();
    }
    else
    {
        // if imgColor is empty build it from gray image 
        if( !imgLeft.empty() ) 
        {
            imgColor = cv::Mat(imgLeft.rows, imgLeft.cols, CV_8UC3);
            cv::cvtColor(imgLeft, imgColor, cv::COLOR_GRAY2BGR);
        }
    }
    if( !imgDepth.empty() ) 
        imgDepth = imgDepth.clone(); 
    
    if( !imgLeft.empty() ) 
    {
        convertCloneGrayImagetoCV8U(imgLeft);
    }
    if( !imgRight.empty() ) 
    {
        convertCloneGrayImagetoCV8U(imgRight);
    }
    
    bIsInitialized = true; 
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::PreProcess()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
    
    if(bIsProcessed) return;
    
    if( bStereo && imgDepth.empty() )
    {
        if(skbNeedRectification)
        {
            std::cout << "PointCloudKeyFrame<PointT>::PreProcess() - performing rectification on-the-fly" << std::endl;
            const Settings* settings = Settings::instance();
            MSG_ASSERT(settings, "Settings not initialized");

            const cv::Mat& M1l = settings->M1l();
            const cv::Mat& M2l = settings->M2l();
            const cv::Mat& M1r = settings->M1r();
            const cv::Mat& M2r = settings->M2r();

            cv::Mat imgLeftRect, imgRightRect;

            std::thread t1([&](){cv::remap(imgLeft, imgLeftRect, M1l, M2l, cv::INTER_LINEAR);});
            std::thread t2([&](){cv::remap(imgRight, imgRightRect, M1r, M2r, cv::INTER_LINEAR);});
            t1.join();
            t2.join();

#if 0 && RERUN_ENABLED
            auto& rec = RerunSingleton::instance();
            rec.log("debug/PCKF/image", rerun::Image(tensor_shape(imgLeft), rerun::TensorBuffer::u8(imgLeft)));                 
            rec.log("debug/PCKF/image_rectified", rerun::Image(tensor_shape(imgLeftRect), rerun::TensorBuffer::u8(imgLeftRect)));     
#endif            
            cv::swap(imgLeft, imgLeftRect);
            cv::swap(imgRight, imgRightRect);
        }

        switch(skStereoLibrary)
        {
            
        case kLibelas: 
#ifdef USE_LIBELAS             
            ProcessStereoLibelas();
#else
            std::cerr << "libelas not supported under this build configuration " << std::endl; 
#endif
            break;
            
        case kLibsgm:
#ifdef USE_LIBSGM             
            ProcessStereoLibsgm();
#else
            std::cerr << "libsgm not supported under this build configuration " << std::endl; 
#endif            
            break;
            
        case kLibOpenCV:
            ProcessStereo();
            break;
            
        case kLibOpenCVCuda:
#ifdef USE_CUDA            
            ProcessStereoCuda();
#else
            std::cerr << "\nWARNING: !opencv cuda not supported under this build configuration!\n" << std::endl; 
#endif           
            break;
            
        default:
            std::cerr << "unknown stereo lib" << std::endl; 
        }
    }
    
    bIsProcessed = true;
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::SetCloud(typename PointCloudKeyFrame<PointT>::PointCloudT::Ptr p_cloud_camera_in)
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
    pCloudCamera = p_cloud_camera_in;

    //imgColor.release();
    //imgDepth.release();

    bCloudReady = true;
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ReleaseColor()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);    
    imgColor.release();
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ReleaseDepth()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);    
    imgDepth.release();
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ReleasePointIndexMat()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);    
    imgPointIndex.release();
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ReleaseLabelMap()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);    
    labelMap.Clear();
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::Release()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);

    imgColor.release();
    imgDepth.release();
    imgLeft.release();
    imgRight.release();
    
    imgPointIndex.release();    
    
    labelMap.Clear();
    
    if (pCloudCamera) pCloudCamera->clear();
    bCloudReady = false;
}
    
// get Ow
template<typename PointT>
Eigen::Vector3f PointCloudKeyFrame<PointT>::GetCameraCenter()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
    return pKF->GetCameraCenter();
}

// get Twc
template<typename PointT>
Sophus::SE3f PointCloudKeyFrame<PointT>::GetCameraPose()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
    return pKF->GetPoseInverse();
}

template<typename PointT>
boost::uint64_t PointCloudKeyFrame<PointT>::GetTimestamp()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);
    return TimeUtils::getTimestampfromSec(pKF->mTimeStamp);
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::Clear()
{
    std::unique_lock<std::mutex> locker(keyframeMutex);

    imgColor.release();
    imgDepth.release();
    imgLeft.release();
    imgRight.release();
    
    imgPointIndex.release();    
    
    labelMap.Clear();
    
    if (pCloudCamera) pCloudCamera->clear();
    bCloudReady = false;
    
    bInMap = false;
    bIsValid = false;
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ProcessStereoLibelas()
{   
    //std::unique_lock<std::mutex> locker(keyframeMutex);    // already locked from Init()
    
#ifdef USE_LIBELAS      
    std::cout << "stereo processing (libelas)" << std::endl;     
    
    TICKLIBELAS("Libelas");     
  
    const float bf = pCamParams? pCamParams->bf : pKF->mbf;

    if(!pElas)
    {
        libelas::Elas::Parameters param;
        param.postprocess_only_left = true;
        param.subsampling = (PointCloudMapping::skDownsampleStep % 2) == 0; 
        //param.filter_adaptive_mean = false;
        //param.ipol_gap_width = 300;

        //const float minZ = pKF->mb; // baseline in meters      
        //param.disp_min = 0;    
        //param.disp_max = pKF->mbf/minZ; // here maxD = fx    
        
        pElas.reset( new libelas::ElasInterface(param));
    }
    const libelas::Elas::Parameters& param = pElas->getParameters();
        
    // get image width and height
    int32_t width, height;
    if(param.subsampling )
    {
        width = (imgLeft.cols/2);
        height = (imgLeft.rows/2);    
    }
    else
    {
        width = imgLeft.cols;
        height = imgLeft.rows;              
    }

    // allocate memory for disparity images
    const int32_t dims[3] = { imgLeft.cols, imgLeft.rows, imgLeft.step[0] }; // bytes per line = width
    //float* D1_data = new float[width*height];
    //float* D2_data = new float[width*height];        
    cv::Mat imgD1 = cv::Mat(height, width, CV_32F);  
    cv::Mat imgD2 = cv::Mat(height, width, CV_32F);        
    if( !imgD1.isContinuous() ) 
    {
        std::cerr << "error cv mat allocation is not continuous " << std::endl; 
        quick_exit(-1);
    }
    
    // inputs: pointers to left (I1) and right (I2) intensity image (uint8, input)
    //         pointers to left (D1) and right (D2) disparity image (float, output)
    //         dims[0] = width of I1 and I2
    //         dims[1] = height of I1 and I2
    //         dims[2] = bytes per line (often equal to width, but allowed to differ)
    //         note: D1 and D2 must be allocated before (bytes per line = width)
    //               if subsampling is not active their size is width x height,
    //               otherwise width/2 x height/2 (rounded towards zero)  
    //void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);    
    pElas->process( (uint8_t*)imgLeft.data, (uint8_t*)imgRight.data, (float*)imgD1.data, (float*)imgD2.data, dims);   
    
    const int subsamplingStep = PointCloudMapping::skDownsampleStep;
    if(param.subsampling )
    {        
        imgDepth = cv::Mat(imgLeft.rows, imgLeft.cols, CV_32F, 0.f);
        for (int m1 = 0, m = 0; m1 < imgD1.rows; m1++, m+=subsamplingStep)
        {
            const float* depth_row_m1 = imgD1.ptr<float>(m1);
            float* depth_row_m  = imgDepth.ptr<float>(m);            
            for (int n1 = 0, n = 0; n1 < imgD1.cols; n1++, n += subsamplingStep)
            {
                const float& d1 = depth_row_m1[n1];                        
                depth_row_m[n] = depth_row_m[n+1] = bf/d1;                
            }            
        }        
    }
    else
    {
        imgDepth = bf/imgD1;
    }
    
    //delete[] D1_data;
    //delete[] D2_data;
    
    // once done release left and right images
    imgLeft.release();
    imgRight.release();
    
    TOCKLIBELAS("Libelas");
    
    SENDALLLIBELAS;        
        
#endif
    
}


template<typename PointT>
void PointCloudKeyFrame<PointT>::ProcessStereoLibsgm()   
{
    //std::unique_lock<std::mutex> locker(keyframeMutex);    // already locked from Init()            

    std::cout << "stereo processing (libsgm)" << std::endl;         
    
    const float bf = pCamParams? pCamParams->bf : pKF->mbf;

#ifdef USE_LIBSGM    
    
    int bits = 8;
    switch (imgLeft.type()) 
    {
    case CV_8UC1: 
        bits = 8; 
        break;
    case CV_16UC1: 
        bits = 16; 
        break;
    default:
        bits = 16; 
        std::cout << "converting images to BW 16 bits" << std::endl; 
        imgLeft.convertTo(imgLeft, CV_16UC1);
        imgRight.convertTo(imgRight, CV_16UC1);
    }
        
    if(!pSgm)
    {
        const bool bDownScale = ((PointCloudMapping::skDownsampleStep % 2) == 0);  
        const int disp_size = 64;
        pSgm.reset( new sgm::StereoSGM(imgLeft.cols, imgLeft.rows, disp_size, bits, 8, sgm::EXECUTE_INOUT_HOST2HOST));
    }
    
    cv::Mat imgDisp(cv::Size(imgLeft.cols, imgLeft.rows), CV_8UC1);    
    pSgm->execute(imgLeft.data, imgRight.data, imgDisp.data);
    
    imgDisp.convertTo(imgDisp, CV_32F);//, 1.f/16.0f);
     
    imgDepth = bf/imgDisp;
       
#endif
    
    // once done release left and right images
    imgLeft.release();
    imgRight.release();       
    
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::ProcessStereo()
{
    //std::unique_lock<std::mutex> locker(keyframeMutex);    // already locked from Init()           

    std::cout << "stereo processing (opencv)" << std::endl;         
    
    const float bf = pCamParams? pCamParams->bf : pKF->mbf;

    if(!pSd)
    {
        const bool bDownScale = ((PointCloudMapping::skDownsampleStep % 2) == 0);  
        StereoParameters params;
        
        params.nMaxDisparity = PLVS2::StereoParameters::kMaxDisparity; 
        params.bDownScale = bDownScale; 
        params.nFilterType = PLVS2::StereoFilterTypes::kWlsConf;        
        pSd.reset( new StereoDisparityCPU());
        pSd->Init(params);
    }
    
    cv::Mat imgDisp;
    pSd->Compute(imgLeft, imgRight, imgDisp);
    
    imgDisp.convertTo(imgDisp, CV_32F, 1.f/16.0f);
    
    imgDepth = bf/imgDisp;
     
    // once done release left and right images
    imgLeft.release();
    imgRight.release();     
}


template<typename PointT>
void PointCloudKeyFrame<PointT>::ProcessStereoCuda()
{
    //std::unique_lock<std::mutex> locker(keyframeMutex);    // already locked from Init() 
        
    const float bf = pCamParams? pCamParams->bf : pKF->mbf;

#ifdef USE_CUDA
 
    std::cout << "stereo processing cv::cuda - need improvements" << std::endl;    
    
    if(!pSd)
    {
        const bool bDownScale = ((PointCloudMapping::skDownsampleStep % 2) == 0);  
        StereoParameters params;
        params.bDownScale = bDownScale; 
        params.nCudaAlgorithmType = StereoCudaAlgorithms::kCudaCSBP; // kCudaBM = 0, kCudaBP, kCudaCSBP
        params.bCudaDisparityFilter = true;
      
        pSd.reset( new StereoDisparityGPU());
        pSd->Init(params);
    }
    
    cv::Mat imgDisp;
    pSd->Compute(imgLeft, imgRight, imgDisp);    
    
    imgDisp.convertTo(imgDisp, CV_32F);    
    
    imgDepth = bf/imgDisp;
        
#endif  
    
    // once done release left and right images
    imgLeft.release();
    imgRight.release();        
}

template<typename PointT>
void PointCloudKeyFrame<PointT>::GetTransformedCloud(const cv::Mat& Twc, typename PointCloudT::Ptr pCloudOut)
{
    std::unique_lock<std::mutex> locker(keyframeMutex);  
    
    Eigen::Isometry3d T = PLVS2::Converter::toSE3Quat(Twc);

// #if !USE_NORMALS
//     pcl::transformPointCloud(*pCloudCamera, *pCloudOut, T.matrix());
// #else
//     pcl::transformPointCloudWithNormals(*pCloudCamera, *pCloudOut, T.matrix());
// #endif
#if 0
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3d, double>(*pCloudCamera, *pCloudOut, T);
#else 
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3f, float>(*pCloudCamera, *pCloudOut, T.cast<float>());
#endif  
}

}
