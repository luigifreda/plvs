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

//#define _DEBUG // for activating debugging in opencv

#include "PointCloudMapChisel.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>

#include <chisel_server/ChiselServer.h>
#include <chisel_server/Conversions.h>

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "KeyFrame.h"

namespace PLVS2
{


template<typename PointT>
PointCloudMapChisel<PointT>::PointCloudMapChisel(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params)
{
    //this->bPerformCarving_ = useCarving_in;

    pChiselServerParams_ = std::make_shared<chisel_server::ChiselServerParams>();
    pChiselServerParams_->chunkSizeX = pChiselServerParams_->chunkSizeY = pChiselServerParams_->chunkSizeZ = 16; 
    pChiselServerParams_->voxelResolution = params->resolution;
    pChiselServerParams_->useColor = true;
    pChiselServerParams_->nearPlaneDist = params->minDepthDistance;
    pChiselServerParams_->farPlaneDist = params->maxDepthDistance;
    pChiselServerParams_->useCarving = params->bUseCarving;
    pChiselServerParams_->carvingDist = 0.05;
    pChiselServerParams_->fusionMode = (int) chisel_server::ChiselServer::FusionMode::PointCloud;

    this->pChiselServer_ = std::make_shared<chisel_server::ChiselServer>(*pChiselServerParams_);
}

template<typename PointT>
void PointCloudMapChisel<PointT>::SetDepthCameraModel(const PointCloudCamParams& params)
{
    this->pChiselServer_->SetDepthCameraInfo(params.fx, params.fy, params.cx, params.cy, params.width, params.height);
}

template<typename PointT>
void PointCloudMapChisel<PointT>::SetColorCameraModel(const PointCloudCamParams& params)
{
    this->pChiselServer_->SetColorCameraInfo(params.fx, params.fy, params.cx, params.cy, params.width, params.height);
}

template<typename PointT>
void PointCloudMapChisel<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, double max_range)
{
    std::cout << "PointCloudMapChisel<PointT>::InsertCloud()" << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    chisel::Transform transform(chisel::Transform::Identity());
    // transform.translation()(0) = Twc.at<float>(0, 3);
    // transform.translation()(1) = Twc.at<float>(1, 3);
    // transform.translation()(2) = Twc.at<float>(2, 3);
    transform.translation() = Twc.translation(); 
    //transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
    transform.linear() = Twc.rotationMatrix();

    this->pChiselServer_->SetPointCloud(*cloud_camera, transform);
    this->pChiselServer_->IntegrateLastPointCloud(false);

    if (pChiselServerParams_->useCarving)
    {
        std::cout << "using carving" << std::endl;
    }
}

template<typename PointT>
void PointCloudMapChisel<PointT>::InsertCloudWithDepth(typename PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, const cv::Mat& depthImage, double max_range)
{
    std::cout << "PointCloudMapChisel<PointT>::InsertCloud() - with depth " << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    chisel::Transform transform(chisel::Transform::Identity());
    // transform.translation()(0) = Twc.at<float>(0, 3);
    // transform.translation()(1) = Twc.at<float>(1, 3);
    // transform.translation()(2) = Twc.at<float>(2, 3);
    transform.translation() = Twc.translation(); 
    //transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
    transform.linear() = Twc.rotationMatrix();

    const int depthWidth = depthImage.cols;
    const int detphHeight = depthImage.rows;
    const int depthStep = depthImage.step;
#if 0
    const float* depthData = depthImage.ptr<float>(0);
    this->pChiselServer_->SetDepthImage(depthData, depthWidth, detphHeight, depthStep, cloud_camera->header.stamp);
#else
    float* depthData = (float *) depthImage.ptr<float>(0);
    this->pChiselServer_->SetDepthImageMemorySharing(depthData, depthWidth, detphHeight, depthStep, cloud_camera->header.stamp);
#endif

    this->pChiselServer_->SetPointCloud(*cloud_camera, transform);
    this->pChiselServer_->IntegrateLastPointCloud(false);

    if (pChiselServerParams_->useCarving)
    {
        std::cout << "using carving" << std::endl;
    }
}

template<typename PointT>
void PointCloudMapChisel<PointT>::InsertDepthScanColor(const cv::Mat& depthImage, const cv::Mat& colorImage, const Sophus::SE3f& Twc, boost::uint64_t timestamp)
{
    std::cout << "PointCloudMapChisel<PointT>::InsertDepthScanColor()" << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    chisel::Transform transform(chisel::Transform::Identity());
    // transform.translation()(0) = Twc.at<float>(0, 3);
    // transform.translation()(1) = Twc.at<float>(1, 3);
    // transform.translation()(2) = Twc.at<float>(2, 3);
    transform.translation() = Twc.translation();
    // transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
    transform.linear() = Twc.rotationMatrix(); 

    if (!depthImage.empty() && !colorImage.empty())
    {

        this->pChiselServer_->SetColorPose(transform);
        this->pChiselServer_->SetDepthPose(transform);

        const int depthWidth = depthImage.cols;
        const int detphHeight = depthImage.rows;
        const int depthStep = depthImage.step;
#if 0
        const float* depthData = depthImage.ptr<float>(0);
        this->pChiselServer_->SetDepthImage(depthData, depthWidth, detphHeight, depthStep, timestamp);
#else
        float* depthData = (float *) depthImage.ptr<float>(0);
        this->pChiselServer_->SetDepthImageMemorySharing(depthData, depthWidth, detphHeight, depthStep, timestamp);
#endif

        const int imgWidth = colorImage.cols;
        const int imgHeight = colorImage.rows;
        const int imgStep = colorImage.step;
        const int imgNumChannels = colorImage.channels();
#if 0
        const uchar* imgData = colorImage.ptr<uchar>(0);
        this->pChiselServer_->SetColorImage(imgData, imgWidth, imgHeight, imgStep, imgNumChannels, timestamp);
#else
        uchar* imgData = (uchar *) colorImage.ptr<uchar>(0);
        this->pChiselServer_->SetColorImageMemorySharing(imgData, imgWidth, imgHeight, imgStep, imgNumChannels, timestamp);
#endif

        this->pChiselServer_->IntegrateLastDepthImage(false);
    }
    else
    {
        std::cout << "PointCloudMapChisel::InsertDepthScanColor() - ERROR: depth and/or color images are emtpy " << std::endl;
    }

    if (pChiselServerParams_->useCarving)
    {
        std::cout << "using carving" << std::endl;
    }
}

template<typename PointT>
void PointCloudMapChisel<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    Sophus::SE3f Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc; 
    
    KeyFramePtr pKF = pData->pPointCloudKeyFrame->pKF;
    uint32_t kfid = pKF->mnId;    
    this->mapKfidPointCloudKeyFrame_[kfid] = pData->pPointCloudKeyFrame;         

    switch (pData->type)
    {
    case PointCloudMapInput<PointT>::kPointCloud:
        //InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_camera, cv::Mat& Twc, double max_range )
        this->InsertCloud(pData->pPointCloudKeyFrame->pCloudCamera, Twc, pData->maxRange);
        break;

    case PointCloudMapInput<PointT>::kColorAndDepthImages:
        //InsertDepthScanColor(cv::Mat& depthImage, cv::Mat& colorImage, cv::Mat& Twc, boost::uint64_t timestamp)
        this->InsertDepthScanColor(pData->pPointCloudKeyFrame->imgDepth, pData->pPointCloudKeyFrame->imgColor, Twc, pData->timestamp);
        break;

    case PointCloudMapInput<PointT>::kPointCloudAndDepthImage:
        this->InsertCloudWithDepth(pData->pPointCloudKeyFrame->pCloudCamera, Twc, pData->pPointCloudKeyFrame->imgDepth, pData->maxRange);
        break;

    default:
        std::cout << "PointCloudMapChisel<PointT>::InsertData() - ERROR - unknown data mode" << std::endl;
        quick_exit(-1);
    }
}

template<typename PointT>
int PointCloudMapChisel<PointT>::UpdateMap()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < udate map 
    if (!this->pPointCloud_) this->pPointCloud_.reset(new PointCloudT());

    std::cout << "\nPointCloudMapChisel - updating mesh" << std::endl;
    this->pChiselServer_->UpdateMesh();
    std::cout << "\nPointCloudMapChisel - getting cloud" << std::endl;
    this->pChiselServer_->GetPointCloud(*(this->pPointCloud_));

    std::cout << "\nPointCloudMapChisel - generated map - size: " << this->pPointCloud_->size() << std::endl;

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapChisel<PointT>::Clear()
{
    std::cout << "PointCloudMapChisel<PointT>::Clear()" << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->pChiselServer_->Reset();

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
}

template<typename PointT>
void PointCloudMapChisel<PointT>::OnMapChange()
{
    //std::cout << "PointCloudMapChisel<PointT>::OnRebuildMap()" << std::endl; 

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset *** " << std::endl;
        this->pChiselServer_->Reset();
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset done! " << std::endl;
    }
}

#if 0
// re-integration by using the method pChiselServer_->IntegrateWorldPointCloud()
template<>
void PointCloudMapChisel<pcl::PointSurfelSegment>::OnMapChange()
{
    //std::cout << "PointCloudMapChisel<PointT>::OnRebuildMap()" << std::endl; 
    
    typedef pcl::PointSurfelSegment PointT;
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef std::unordered_map<uint32_t, typename PointCloudMap<PointT>::PointCloudT> MapKfidCloud;    

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset *** " << std::endl;
        this->pChiselServer_->Reset();
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset done! " << std::endl;
    }
    
    if(this->bKfAdjustmentOnSparseMapChange_)
    {
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - point cloud KF adjustment" << std::endl;
        
        this->UpdateMap(); // update the pointcloud 
                
        // KF Adjustment of the map 
        //      * iterate over all points of the map:
        //          - group them in pairs (KFID, vector of points with kfid==KFID)  (?couldn't we maintain this when we integrate the new sensed cloud in AM?)        
        //      * correct each group of KF points according to the new position of the corresponding KF 
        //      * re-integrate all the group of KF points in the new map
        
        MapKfidCloud mapKfidToPointCloud;        
        for(size_t ii=0,iiEnd=this->pPointCloud_->size(); ii<iiEnd; ii++)
        {
            const pcl::PointSurfelSegment& point = this->pPointCloud_->points[ii];
            mapKfidToPointCloud[point.kfid].points.push_back(point); 
        }
        
        // once recollected all the points, let's clear the map 
        PointCloudMap<PointT>::Clear();
        this->pChiselServer_->Reset();
                
        PointCloudT::Ptr pCloudWorldNew( new PointCloudT );
        
        PointCloudT::Ptr pkfCloudWorldNew( new PointCloudT );
        PointCloudT& kfCloudWorldNew = *pkfCloudWorldNew;
        //const cv::Mat identity = cv::Mat::eye(4,4,CV_32F);
        MapKfidCloud::iterator itc=mapKfidToPointCloud.begin(), itcEnd=mapKfidToPointCloud.end();
        for(;itc!=itcEnd; itc++)
        {
            const uint32_t kfid = itc->first;
            PointCloudT& kfCloudWorld = itc->second; 
            
            typename PointCloudKeyFrameT::Ptr pcKF = mapKfidPointCloudKeyFrame_[kfid];
            if(!pcKF->bIsValid) continue; 
            
            KeyFramePtr pKF = pcKF->pKF;
            if(pKF->isBad()) continue;
            
            // let's correct the cloud 
            const Sophus::SE3f& TwcIntegration = pcKF->TwcIntegration; // pose at the last time of integration 
            
            //cv::Mat TcwIntegration = cv::Mat::eye(4,4,CV_32F);
            // invert by taking into account the structure of the homogeneous transformation matrix
            // cv::Mat RcwIntegration =  TwcIntegration.rowRange(0,3).colRange(0,3).t();
            // cv::Mat tcwIntegration = -RcwIntegration*TwcIntegration.rowRange(0,3).col(3);
            // RcwIntegration.copyTo(TcwIntegration.rowRange(0,3).colRange(0,3));
            // tcwIntegration.copyTo(TcwIntegration.rowRange(0,3).col(3));
            Sophus::SE3f TcwIntegration = TwcIntegration.inverse();
            
            Sophus::SE3f TwcNew =  pKF->GetPoseInverse();  // new corrected pose 
            
            // let's compute the correction transformation 
            //cv::Mat Twnwo= TwcNew * TwcIntegration.inv(); // from world old to world new             
            Sophus::SE3f Twnwo = TwcNew * TcwIntegration; // from world old to world new 
            
            // check if the transformation is "big" enough otherwise do not re-transform the cloud 
            //double norm = cv::norm(Twnwo - identity);
            double norm = (Twnwo.matrix3x4() - Sophus::SE3f().matrix3x4()).norm();
            std::cout << "norm: " << norm << std::endl; 
            if( norm > kNormThresholdForEqualMatrices)
            {            
                this->TransformCameraCloudInWorldFrame(kfCloudWorld, Converter::toIsometry3d(Twnwo), kfCloudWorldNew);   
            }
            else
            {
                kfCloudWorldNew.swap(kfCloudWorld);
            }
            
            // update integration pose  
            pcKF->TwcIntegration = TwcNew;
            
            // reintegrate the cloud           
            *(pCloudWorldNew) += kfCloudWorldNew; // push the corrected cloud               
            //this->ReinsertCloud(pkfCloudWorldNew);         
        }
        
        chisel::Transform transform(chisel::Transform::Identity());
        this->pChiselServer_->IntegrateWorldPointCloud(*pCloudWorldNew, transform);           
                           
        this->UpdateMap(); // filter and update timestamp 
        
        /// < update timestamp !
        //this->UpdateMapTimestamp();
         
    }    
}
#endif 

#define REUSE_LAST_KFID_ON_INVALID_KF 1

template<>
void PointCloudMapChisel<pcl::PointSurfelSegment>::OnMapChange()
{
    //std::cout << "PointCloudMapChisel<PointT>::OnRebuildMap()" << std::endl; 
    
    typedef pcl::PointSurfelSegment PointT;
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef chisel::MapKfidRt MapKfidRt;    

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset *** " << std::endl;
        this->pChiselServer_->Reset();
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - chisel reset done! " << std::endl;
    }
    
    if(this->pPointCloudMapParameters_->bCloudDeformationOnSparseMapChange)
    {
        std::cout << "PointCloudMapChisel<PointT>::OnMapChange() - point cloud KF adjustment" << std::endl;
        
        this->UpdateMap(); // update the pointcloud with the last pushed scan clouds 
        
        MapKfidRt mapKfidToRt;  // kfid -> R , t for volumetric deformation 
        
        // let's clear the map since we rebuild it after chisel deformation 
        PointCloudMap<PointT>::Clear();

        const cv::Mat identity = cv::Mat::eye(4,4,CV_32F);
        
        uint32_t lastKfid = 0;
        Sophus::SE3f lastTwnwo; // = cv::Mat::eye(4,4,CV_32F);
        
        auto itc=mapKfidPointCloudKeyFrame_.begin(), itcEnd=mapKfidPointCloudKeyFrame_.end();
        for(;itc!=itcEnd; itc++)
        {
            const uint32_t kfid = itc->first;            
            typename PointCloudKeyFrameT::Ptr pcKF = itc->second;
            
            Sophus::SE3f Twnwo;
                    
            KeyFramePtr pKF = pcKF->pKF;
            if(pcKF->bIsValid && !pKF->isBad())
            {            
                // let's compute the transformation for kfid 
                const Sophus::SE3f& TwcIntegration = pcKF->TwcIntegration; // pose at the last time of integration 

                // invert TwcIntegration by taking into account the structure of the homogeneous transformation matrix                
                // cv::Mat TcwIntegration = cv::Mat::eye(4,4,CV_32F); // inverse of TwcIntegration
                // cv::Mat RcwIntegration =  TwcIntegration.rowRange(0,3).colRange(0,3).t();
                // cv::Mat tcwIntegration = -RcwIntegration*TwcIntegration.rowRange(0,3).col(3);
                // RcwIntegration.copyTo(TcwIntegration.rowRange(0,3).colRange(0,3));
                // tcwIntegration.copyTo(TcwIntegration.rowRange(0,3).col(3));
                Sophus::SE3f TcwIntegration = TwcIntegration.inverse();

                Sophus::SE3f TwcNew =  pKF->GetPoseInverse();  // new corrected pose 

                // let's compute the correction transformation 
                //cv::Mat Twnwo= TwcNew * TwcIntegration.inv(); // from world old to world new             
                Twnwo = TwcNew * TcwIntegration; // from world old (last integration) to world new 

                // check if the transformation is "big" enough otherwise do not re-transform the cloud 
                //double norm = cv::norm(Twnwo - identity);
                double norm = (Twnwo.matrix3x4() - Sophus::SE3f().matrix3x4()).norm();
                std::cout << "kfid: " << kfid << ", norm: " << norm << std::endl; 

                // update integration pose  
                pcKF->TwcIntegration = TwcNew;
                
                lastTwnwo = Twnwo;                
                lastKfid = kfid; 
            }
            else
            {
#if REUSE_LAST_KFID_ON_INVALID_KF                
                std::cout << "reused kfid: " << lastKfid << std::endl;
                Twnwo = lastTwnwo;
#else
                continue; 
#endif                
            }
            
            chisel::TransformRt& Rt = mapKfidToRt[kfid];
            // Rt.R << Twnwo(0,0), Twnwo(0,1), Twnwo(0,2), 
            //         Twnwo(1,0), Twnwo(1,1), Twnwo(1,2),
            //         Twnwo(2,0), Twnwo(2,1), Twnwo(2,2);
            Rt.R = Twnwo.rotationMatrix();
            
            //Rt.t << Twnwo(0,3), Twnwo(1,3), Twnwo(2,3);
            Rt.t = Twnwo.translation();
            
            // reintegrate the cloud           
            //*(pCloudWorldNew) += kfCloudWorldNew; // push the corrected cloud               
            //this->ReinsertCloud(pkfCloudWorldNew);         
        }
        
#if 0        
        chisel::Transform transform(chisel::Transform::Identity());
        this->pChiselServer_->IntegrateWorldPointCloud(*pCloudWorldNew, transform);   
#else        
        this->pChiselServer_->Deform(mapKfidToRt);
#endif        
                           
        this->UpdateMap(); // filter and update timestamp 
        
        /// < update timestamp !
        //this->UpdateMapTimestamp();
         
    }    
}

static std::string removeFileNameExtension(const std::string& fileName)
{
    std::size_t pos = fileName.rfind('.');
    if (pos == std::string::npos) return fileName;
    std::string resString(fileName.substr(0, pos));
    return resString;
}

template<typename PointT>
void PointCloudMapChisel<PointT>::SaveMap(const std::string& filename)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    //PointCloudMap<PointT>::SaveMap(filename);
    PointCloudMap<PointT>::SaveTriangleMeshMap(filename);        

#if 0    
    std::string filename_mesh = removeFileNameExtension(filename);
    filename_mesh = filename_mesh + "_mesh.ply";

    this->pChiselServer_->SaveMesh(filename_mesh);
#endif 
    
}


template<typename PointT>
bool PointCloudMapChisel<PointT>::LoadMap(const std::string& filename)
{
    std::cout << "PointCloudMapChisel<PointT>::LoadMap() - loading..." << std::endl;
    
    if( PointCloudMap<PointT>::LoadMap(filename) )
    {
        std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

        chisel::Transform transform(chisel::Transform::Identity());
        this->pChiselServer_->IntegrateWorldPointCloud(*(this->pPointCloud_), transform);

        this->UpdateMap();
        std::cout << "PointCloudMapChisel<PointT>::LoadMap() - done " << std::endl;           
        return true;
    }
    else
    {
        return false;
    }
}




} //namespace PLVS2