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


#include "PointCloudMapVoxblox.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>

#define COMPILE_WITHOUT_ROS
#include <voxblox_ros/tsdf_server.h>

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"


namespace PLVS2
{

template<typename PointT>
std::string PointCloudMapVoxblox<PointT>::skIntegrationMethod = "fast"; /// < simple, merged, fast 


template<typename PointT>
PointCloudMapVoxblox<PointT>::PointCloudMapVoxblox(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params)
{
    //this->bPerformCarving_ = useCarving_in;

    voxblox::TsdfMap::Config tsdfMapConfig;
    tsdfMapConfig.tsdf_voxel_size = params->resolution;
    tsdfMapConfig.tsdf_voxels_per_side = 16; 
    
    voxblox::TsdfIntegratorBase::Config  tsdfIntegratorBaseConfig;   
    tsdfIntegratorBaseConfig.default_truncation_distance = 0.1;
    tsdfIntegratorBaseConfig.max_weight = 10000.0;
    tsdfIntegratorBaseConfig.voxel_carving_enabled = params->bUseCarving;
    tsdfIntegratorBaseConfig.min_ray_length_m = 0.1;
    tsdfIntegratorBaseConfig.max_ray_length_m = 5.0;
    tsdfIntegratorBaseConfig.use_const_weight = false;
    tsdfIntegratorBaseConfig.allow_clear = true;
    tsdfIntegratorBaseConfig.use_weight_dropoff = true;
    tsdfIntegratorBaseConfig.use_sparsity_compensation_factor = false;
    tsdfIntegratorBaseConfig.sparsity_compensation_factor = 1.0f;
    // merge integrator specific
    tsdfIntegratorBaseConfig.enable_anti_grazing = false;
    // fast integrator specific
    tsdfIntegratorBaseConfig.start_voxel_subsampling_factor = 2.0f;
    tsdfIntegratorBaseConfig.max_consecutive_ray_collisions = 2;
    tsdfIntegratorBaseConfig.clear_checks_every_n_frames = 1;

    std::string integration_method = PointCloudMapVoxblox<PointT>::skIntegrationMethod; /// < simple, merged, fast 
    
    this->pTsdfServer_ = std::make_shared<voxblox::TsdfServer>(tsdfMapConfig,tsdfIntegratorBaseConfig, integration_method);
}


template<typename PointT>
void PointCloudMapVoxblox<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_camera, const Sophus::SE3f& Twc, double max_range)
{
    std::cout << "PointCloudMapVoxblox<PointT>::InsertCloud()" << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    voxblox::IsometryTransform transform(voxblox::IsometryTransform::Identity());
    // transform.translation()(0) = Twc.at<float>(0, 3);
    // transform.translation()(1) = Twc.at<float>(1, 3);
    // transform.translation()(2) = Twc.at<float>(2, 3);
    transform.translation() = Twc.translation(); 
    //transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
    transform.linear() = Twc.rotationMatrix();
    
    this->pTsdfServer_->insertPointCloud(*cloud_camera, transform); 

    if (this->pTsdfServer_->getIntegratorConfig().voxel_carving_enabled)
    {
        std::cout << "using carving" << std::endl;
    }
}
//
//template<typename PointT>
//void PointCloudMapVoxblox<PointT>::InsertCloudWithDepth(typename PointCloudT::ConstPtr cloud_camera, const cv::Mat& Twc, const cv::Mat& depthImage, double max_range)
//{
//    std::cout << "PointCloudMapVoxblox<PointT>::InsertCloud() - with depth " << std::endl;
//
//    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
//
//    chisel::Transform transform(chisel::Transform::Identity());
//    transform.translation()(0) = Twc.at<float>(0, 3);
//    transform.translation()(1) = Twc.at<float>(1, 3);
//    transform.translation()(2) = Twc.at<float>(2, 3);
//    transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
//
//    const int depthWidth = depthImage.cols;
//    const int detphHeight = depthImage.rows;
//    const int depthStep = depthImage.step;
//#if 0
//    const float* depthData = depthImage.ptr<float>(0);
//    this->pChiselServer_->SetDepthImage(depthData, depthWidth, detphHeight, depthStep, cloud_camera->header.stamp);
//#else
//    float* depthData = (float *) depthImage.ptr<float>(0);
//    this->pChiselServer_->SetDepthImageMemorySharing(depthData, depthWidth, detphHeight, depthStep, cloud_camera->header.stamp);
//#endif
//
//    this->pChiselServer_->SetPointCloud(*cloud_camera, transform);
//    this->pChiselServer_->IntegrateLastPointCloud(false);
//
//    if (pChiselServerParams_->useCarving)
//    {
//        std::cout << "using carving" << std::endl;
//    }
//}

template<typename PointT>
void PointCloudMapVoxblox<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    Sophus::SE3f Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc;     

    switch (pData->type)
    {
    case PointCloudMapInput<PointT>::kPointCloud: 
        //InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_camera, cv::Mat& Twc, double max_range )
        this->InsertCloud(pData->pPointCloudKeyFrame->pCloudCamera, Twc, pData->maxRange);
        break;

    default:
        std::cout << "PointCloudMapVoxblox<PointT>::InsertData() - ERROR - unknown data mode" << std::endl;
        quick_exit(-1);
    }
}

template<typename PointT>
int PointCloudMapVoxblox<PointT>::UpdateMap()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < udate map 
    if (!this->pPointCloud_) this->pPointCloud_.reset(new PointCloudT());

    std::cout << "\nPointCloudMapVoxblox - updating mesh" << std::endl;
    this->pTsdfServer_->updateMesh();
    
    std::cout << "\nPointCloudMapVoxblox - getting cloud" << std::endl;
    this->pTsdfServer_->getMeshAsPointcloud(*(this->pPointCloud_));

    std::cout << "\nPointCloudMapVoxblox - generated map - size: " << this->pPointCloud_->size() << std::endl;

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapVoxblox<PointT>::Clear()
{
    std::cout << "PointCloudMapVoxblox<PointT>::Clear()" << std::endl;

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->pTsdfServer_->clear();

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
}

template<typename PointT>
void PointCloudMapVoxblox<PointT>::OnMapChange()
{
    //std::cout << "PointCloudMapVoxblox<PointT>::OnRebuildMap()" << std::endl; 

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxblox<PointT>::OnMapChange() - voxblox reset *** " << std::endl;
        this->pTsdfServer_->clear();
        std::cout << "PointCloudMapVoxblox<PointT>::OnMapChange() - voxblox reset done! " << std::endl;
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
void PointCloudMapVoxblox<PointT>::SaveMap(const std::string& filename)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    //PointCloudMap<PointT>::SaveMap(filename);
    PointCloudMap<PointT>::SaveTriangleMeshMap(filename);    

    std::string filename_mesh = removeFileNameExtension(filename);
    filename_mesh = filename_mesh + ".proto";

    this->pTsdfServer_->saveMap(filename_mesh);
}


template<typename PointT>
bool PointCloudMapVoxblox<PointT>::LoadMap(const std::string& filename)
{
    std::cout << "PointCloudMapVoxblox<PointT>::LoadMap() - loading..." << std::endl;
    
    if( Utils::hasSuffix(filename,".proto") )
    {
        std::cout << "loading proto map ... " << std::endl; 
        return this->pTsdfServer_->loadMap(filename);
    }
    else if( PointCloudMap<PointT>::LoadMap(filename) )
    {

        std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

        voxblox::IsometryTransform transform(voxblox::IsometryTransform::Identity());

        this->pTsdfServer_->insertWorldPointCloud(*(this->pPointCloud_), transform ); 

        if (this->pTsdfServer_->getIntegratorConfig().voxel_carving_enabled)
        {
            std::cout << "using carving" << std::endl;
        }

        this->UpdateMap();
        std::cout << "PointCloudMapVoxblox<PointT>::LoadMap() - done " << std::endl;    
        return true;         
    }
    else
    {
        return false; 
    }
    
}





} //namespace PLVS2