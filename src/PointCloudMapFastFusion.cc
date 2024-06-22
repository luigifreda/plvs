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

#include "PointCloudMapFastFusion.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>


#ifdef USE_FASTFUSION
#include <camerautils/camerautils.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <fusion/mesh.hpp>
#endif 

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"



namespace PLVS2
{

template<typename PointT>
PointCloudMapFastFusion<PointT>::PointCloudMapFastFusion(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params)
{
    useColor = true;

    threadMeshing = false;
    performIncrementalMeshing = true;
    depthConstistencyChecks = 0;

    scale = params->resolution;
    //distanceThreshold = sqrt(3) * resolution_in;
    distanceThreshold = 2 * sqrt(3) * params->resolution;

    //this->bPerformCarving_ = useCarving_in;

    Init();
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::Init()
{
#ifdef USE_FASTFUSION    
    pFusion_.reset(new FusionMipMapCPU(0, 0, 0, scale, distanceThreshold, 0, useColor));

    pFusion_->setThreadMeshing(threadMeshing);
    pFusion_->setDepthChecks(depthConstistencyChecks);
    pFusion_->setIncrementalMeshing(performIncrementalMeshing);

    pCurrentMeshInterleaved_.reset(new MeshInterleaved(3));
#endif
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::SetDepthCameraModel(const PointCloudCamParams& params)
{
    depthCameraModel_ = params;
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::SetColorCameraModel(const PointCloudCamParams& params)
{
    colorCameraModel_ = params;
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    
#ifdef USE_FASTFUSION  
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    CameraInfo cameraInfo;
    cv::Mat intrinsic = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    // Intrinsic Parameters
    intrinsic.at<double>(0, 0) = depthCameraModel_.fx;
    intrinsic.at<double>(1, 1) = depthCameraModel_.fy;
    intrinsic.at<double>(0, 2) = depthCameraModel_.cx;
    intrinsic.at<double>(1, 2) = depthCameraModel_.cy;
    cameraInfo.setIntrinsic(intrinsic);

    Sophus::SE3f Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc; 
        
    //cv::Mat Twc_double;
    //Twc_float.convertTo(Twc_double, CV_64F);
    cv::Mat cvTwc_double = Converter::toCvSE3d(Twc);
    cameraInfo.setExtrinsic(cvTwc_double);

    cv::Mat depthImage;
    float scale = 5000.0f;
    pData->pPointCloudKeyFrame->imgDepth.convertTo(depthImage, CV_16U, scale);

    //std::cout << "depth:  " <<depthImage << std::endl;

    switch (pData->type)
    {

    case PointCloudMapInput<PointT>::kColorAndDepthImages:
        //pFusion_->addMap(pData->pPointCloudKeyFrame->imgDepth, cameraInfo, pData->pPointCloudKeyFrame->imgColor, 1.0f / pData->imageDepthScale, pData->maxRange);
        pFusion_->addMap(depthImage, cameraInfo, pData->pPointCloudKeyFrame->imgColor, 1.0f / scale, pData->maxRange);
        break;

    default:
        std::cout << "PointCloudMapFastFusion<PointT>::InsertData() - ERROR - unknown data mode" << std::endl;
        quick_exit(-1);
    }

    pFusion_->updateMeshes();
    
#endif    
}

#if !USE_NORMALS

template<typename PointT>
int PointCloudMapFastFusion<PointT>::UpdateMap()
{
    
#ifdef USE_FASTFUSION  
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < udate map 

    //pFusion_->updateMeshes();

    *pCurrentMeshInterleaved_ = pFusion_->getMeshInterleavedMarchingCubes();

    /// < update pointcloud

    //PointCloudMap<PointT>::ResetPointCloud();
    if (!this->pPointCloud_) this->pPointCloud_.reset(new PointCloudT());

    std::vector<Vertex3f>& vertices = pCurrentMeshInterleaved_->vertices;
    std::vector<Color3b>& colors = pCurrentMeshInterleaved_->colors;
    std::vector<Vertex3f>& normals = pCurrentMeshInterleaved_->normals;

    typename PointCloudMap<PointT>::PointCloudT& cloud = *(this->pPointCloud_);
    cloud.resize(vertices.size());
    for (size_t ii = 0, iiEnd=vertices.size(); ii < iiEnd; ii++)
    {
        cloud[ii].x = vertices[ii].x;
        cloud[ii].y = vertices[ii].y;
        cloud[ii].z = vertices[ii].z;

        // invert colors for displaying the cloud in openGL
        cloud[ii].r = colors[ii].b;
        cloud[ii].g = colors[ii].g;
        cloud[ii].b = colors[ii].r;
    }

    /// < update timestamp !
    this->UpdateMapTimestamp();

    std::cout << "\nPointCloudMapFastFusion - generated map - size: " << this->pPointCloud_->size() << ", num faces: " << pCurrentMeshInterleaved_->faces.size() << std::endl;

    return this->pPointCloud_->size();
    
#endif
    
}

#else // if !USE_NORMALS

template<typename PointT>
int PointCloudMapFastFusion<PointT>::UpdateMap()
{
    
#ifdef USE_FASTFUSION      
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    /// < udate map 

    //pFusion_->updateMeshes();

    *pCurrentMeshInterleaved_ = pFusion_->getMeshInterleavedMarchingCubes();

    /// < update pointcloud

    //PointCloudMap<PointT>::ResetPointCloud();
    if (!this->pPointCloud_) this->pPointCloud_.reset(new PointCloudT());

    std::vector<Vertex3f>& vertices = pCurrentMeshInterleaved_->vertices;
    std::vector<Color3b>& colors = pCurrentMeshInterleaved_->colors;
    std::vector<unsigned int>& faces = pCurrentMeshInterleaved_->faces;
    //std::vector<Vertex3f>& normals = pCurrentMeshInterleaved_->normals;

    const size_t numVertices = vertices.size();
    const size_t numFaces = faces.size();
    //const int numVerticesXFace = pCurrentMeshInterleaved_->_verticesPerFace;

    std::cout << "num faces: " << numFaces << ", num vertices: " << numVertices << std::endl;

    typename PointCloudMap<PointT>::PointCloudT& cloud = *(this->pPointCloud_);
    cloud.resize(vertices.size());

    for (size_t ii = 0; ii < numVertices; ii++)
    {
        cloud[ii].x = vertices[ii].x;
        cloud[ii].y = vertices[ii].y;
        cloud[ii].z = vertices[ii].z;

        // invert colors for displaying the cloud in openGL
        cloud[ii].r = colors[ii].b;
        cloud[ii].g = colors[ii].g;
        cloud[ii].b = colors[ii].r;
    }

    //float n1_old = 0, n2_old = 0, n3_old = 0; 
    //bool first = true; 
    for (unsigned int f = 0; f < numFaces; f += 3)
    {
        const size_t ii = faces[f];
        if (ii + 2 >= numVertices) 
        {
            std::cerr << " ERROR: face " << ii << " with illegal index! " << std::endl;
            continue;
        }
        const float d1x = vertices[ii + 0].x - vertices[ii + 2].x;
        const float d1y = vertices[ii + 0].y - vertices[ii + 2].y;
        const float d1z = vertices[ii + 0].z - vertices[ii + 2].z;
        const float d2x = vertices[ii + 1].x - vertices[ii + 2].x;
        const float d2y = vertices[ii + 1].y - vertices[ii + 2].y;
        const float d2z = vertices[ii + 1].z - vertices[ii + 2].z;
        float n1 = d1y*d2z - d1z*d2y;
        float n2 = d1z*d2x - d1x*d2z;
        float n3 = d1x*d2y - d1y*d2x;
        const float length = sqrtf(n1 * n1 + n2 * n2 + n3 * n3);
        n1 /= length;
        n2 /= length;
        n3 /= length;
#if 0
        if (first)
        {
            first = false;
        }
        else
        {
            /// < FIXME: this flipping is not really working (continguous face indexes do not identify contingous faces?)
            if (n1 * n1_old + n2 * n2_old + n3 * n3_old < 0)
            {
                n1 = -n1;
                n2 = -n2;
                n3 = -n3;
            }
        }
        n1_old = n1;
        n2_old = n2;
        n3_old = n3;
#endif
        cloud[ii].normal_x = cloud[ii + 1].normal_x = cloud[ii + 2].normal_x = n1;
        cloud[ii].normal_y = cloud[ii + 1].normal_y = cloud[ii + 2].normal_y = n2;
        cloud[ii].normal_z = cloud[ii + 1].normal_z = cloud[ii + 2].normal_z = n3;
    }

    /// < update timestamp !
    this->UpdateMapTimestamp();

    std::cout << "\nPointCloudMapFastFusion - generated map - size: " << this->pPointCloud_->size() << ", num faces: " << pCurrentMeshInterleaved_->faces.size() << std::endl;

    return this->pPointCloud_->size();

#endif
    
}

#endif //if !USE_NORMALS

template<typename PointT>
void PointCloudMapFastFusion<PointT>::Clear()
{
#ifdef USE_FASTFUSION    
    std::cout << "PointCloudMapFastFusion<PointT>::Clear()" << std::endl;
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
    
    //if(PointCloudMap<PointT>::pPointCloud_->empty()) return; 
    
    pFusion_.reset();
    pCurrentMeshInterleaved_.reset();
    
    ///  < FIXME: this Init() may generate a CRASH (there could be a problem in FusionMipMapCPU destructor)    
    Init(); 

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
#endif    
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::OnMapChange()
{
#ifdef USE_FASTFUSION      
    //std::cout << "PointCloudMapChisel<PointT>::OnRebuildMap()" << std::endl; 

    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapFastFusion<PointT>::OnMapChange() - reset " << std::endl;
        pFusion_.reset();
        pCurrentMeshInterleaved_.reset();
        
        ///  < FIXME: this Init() may generate a CRASH (there could be a problem in FusionMipMapCPU destructor)
        Init();
    }
#endif     
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::GetMapWithTimeout(typename PointCloudT::Ptr& pCloud, typename PointCloudT::Ptr& pCloudUnstable,
                                                        std::vector<unsigned int>& faces, const std::chrono::milliseconds& timeout, bool copyUnstable)
{

#ifdef USE_FASTFUSION  
    std::cout << "PointCloudMapFastFusion<PointT>::GetMapWithTimeout()  *** " << std::endl;
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_, timeout);
    if (lck.owns_lock())
    {
        pCloud = this->pPointCloud_ ? this->pPointCloud_->makeShared() : 0; // deep copy
        if (copyUnstable)
        {
            pCloudUnstable = this->pPointCloudUnstable_ ? this->pPointCloudUnstable_->makeShared() : 0; // deep copy
        }
        faces = pCurrentMeshInterleaved_->faces;
        std::cout << "\nPointCloudMapFastFusion - got map - size: " << this->pPointCloud_->size() <<", num faces: " << pCurrentMeshInterleaved_->faces.size() << std::endl;
    }
    else
    {
        pCloud = 0;
        pCloudUnstable = 0;
        faces.clear();
    }
    
#endif 
    
}

static std::string removeFileNameExtension(const std::string& fileName)
{
    std::size_t pos = fileName.rfind('.');
    if (pos == std::string::npos) return fileName;
    std::string resString(fileName.substr(0, pos));
    return resString;
}

template<typename PointT>
void PointCloudMapFastFusion<PointT>::SaveMap(const std::string& filename)
{
    
#ifdef USE_FASTFUSION  
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    PointCloudMap<PointT>::SaveMap(filename);

    std::string filename_mesh = removeFileNameExtension(filename);
    filename_mesh = filename_mesh + "_mesh.ply";

    pCurrentMeshInterleaved_->writePLY(filename_mesh);
    
#endif
    
}

template<typename PointT>
bool PointCloudMapFastFusion<PointT>::LoadMap(const std::string& filename)
{
#ifdef USE_FASTFUSION  

    std::cout << "PointCloudMapFastFusion<PointT>::LoadMap() - loading..." << std::endl;
    
    if(PointCloudMap<PointT>::LoadMap(filename))
    {

        std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

        // < TODO: actually insert point cloud in

        std::cout << "PointCloudMapFastFusion<PointT>::LoadMap() - NOT YET IMPLEMENTED " << std::endl;    
        return true;
    }
    else
    {
        return false; 
    }
    
    
#endif    
}



} //namespace PLVS2
