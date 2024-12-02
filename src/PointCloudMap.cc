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

#include "PointCloudMap.h"

#include "PointUtils.h"

#include <boost/make_shared.hpp>

#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "KeyFrame.h"
#include "Map.h"
#include "PointCloudUtils.h"


namespace PLVS2
{

template<typename PointT>
const double PointCloudMap<PointT>::kNormThresholdForEqualMatrices = 1e-5; 

template<typename PointT>
PointCloudMap<PointT>::PointCloudMap(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params):
mpMap(pMap), pPointCloudMapParameters_(params), lastTimestamp_(0), bMapUpdated_(false) 
{
    MSG_ASSERT(mpMap!=NULL,"PointCloudMap should be initialized with a valid Map!");
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10        
    pPointCloud_ = boost::make_shared< PointCloudT >();
    pPointCloudUnstable_ = boost::make_shared< PointCloudT >();
#else 
    pPointCloud_ = std::make_shared< PointCloudT >();
    pPointCloudUnstable_ = std::make_shared< PointCloudT >();
#endif
}

template<typename PointT>
void PointCloudMap<PointT>::SetCorrespondingSparseMap(Map* pMap) 
{ 
    MSG_ASSERT(mpMap!=NULL,"PointCloudMap should be initialized with a valid Map!");
    
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);  
    mpMap = pMap; 
}

template<typename PointT>
Map* PointCloudMap<PointT>::GetCorrespodingSparseMap() 
{ 
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);     
    return mpMap; 
}
//
//template<typename PointT>
//void PointCloudMap<PointT>::InsertKeyFrame(typename PointCloudKeyFrame<PointT>::Ptr pcKeyFrame)
//{
//    KeyFramePtr& pKF = pcKeyFrame->pKF;
//    cout << "receive a keyframe, id = " << pKF->mnId << endl;
//    if(baseKeyframeId_< 0) baseKeyframeId_ = pKF->mnId; 
//
//    unique_lock<mutex> lck(keyframesMutex_);
//
//    //pcKeyFrame->Init(); // N.B.: no memory sharing for color and depth  
//    
//    if(pcKeyframes_.empty()) 
//    {
//        //mnInitKFid_ = pKF->mnId;
//        mpKFinitial_ = pcKeyFrame;
//        //mpKFlowerID_ = pcKeyFrame;        
//    }
////    if(pKF->mnId > mnMaxKFid_)
////    {
////        mnMaxKFid_ = pKF->mnId;
////    }
////    if(pKF->mnId < mpKFlowerID_->pKF->mnId)
////    {
////        mpKFlowerID_ = pcKeyFrame;
////    }    
//    
//    pcKeyframesIn_.push_back(pcKeyFrame);
//
//    bKeyframesAvailable_ = true;
//    
////    if (pcKeyframesIn_.size() >= numKeyframesToQueueBeforeProcessing_)
////    {
////        keyFramesCond_.notify_one();
////    }       
//}

template<typename PointT>
long unsigned int PointCloudMap<PointT>::GetId()
{
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);    
    return mpMap->GetId();
}

template<typename PointT>
long unsigned int PointCloudMap<PointT>::GetInitKFid()
{
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);
    return mpMap->GetInitKFid();
}

template<typename PointT>
long unsigned int PointCloudMap<PointT>::GetMaxKFid()
{
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);
    return mpMap->GetMaxKFid();
}

//template<typename PointT>
//typename PointCloudKeyFrame<PointT>::Ptr PointCloudMap<PointT>::GetOriginKF()
//{
//    return mpKFinitial_;
//}

template<typename PointT>
bool PointCloudMap<PointT>::IsBad()
{
    std::unique_lock<std::recursive_timed_mutex> lock(pointCloudMutex_);    
    return mpMap->IsBad();    
}

//template<typename PointT>
//void PointCloudMap<PointT>::SetBad()
//{
//    mbBad_ = true;    
//}


template<typename PointT>
void PointCloudMap<PointT>::SetCurrentMap()
{
    mIsInUse_ = true;
}

template<typename PointT>
void PointCloudMap<PointT>::SetStoredMap()
{
    mIsInUse_ = false;
}

//// reset the full data structure
//template<typename PointT>
//void PointCloudMap<PointT>::Reset()
//{
//    std::cout << "PointCloudMap::Reset() - start " << std::endl;
//
//    unique_lock<mutex> lck(keyframesMutex_); // block the insertion of new frames
//
//    bKeyframesAvailable_ = false;
//    bKeyframesToInsertInMap_ = false; 
//    
//    pcKeyframesIn_.clear();
//
//    {
//        std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);
//        this->Clear();
//    }
//
//    {
//        std::unique_lock<std::mutex> lck(pcKeyframesMutex_);
//        pcKeyframes_.clear();
//        lastKeyframeIndex_ = 0;
//        
//        pcKeyframesToReinsert_.clear();
//    }
//    std::cout << "PointCloudMap::Reset() - end" << std::endl;    
//}

// clear the point cloud 
template<typename PointT>
void PointCloudMap<PointT>::Clear()
{
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);
    if (pPointCloud_) pPointCloud_->clear();
}

template<typename PointT>
void PointCloudMap<PointT>::ResetPointCloud()
{
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);
    if (pPointCloud_)
    {
        pPointCloud_->clear();
    }
    else
    {
        pPointCloud_.reset(new PointCloudT());
    }
    //faces_.clear();
}

template<typename PointT>
void PointCloudMap<PointT>::UpdateMapTimestamp()
{
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);
    if (pPointCloud_) pPointCloud_->header.stamp = TimeUtils::getTimestamp();
    if (pPointCloudUnstable_) pPointCloudUnstable_->header.stamp = (pPointCloud_ ? pPointCloud_->header.stamp : 0);

    bMapUpdated_ = true;
}

template<typename PointT>
void PointCloudMap<PointT>::TransformCameraCloudInWorldFrame(typename PointCloudT::ConstPtr pCloudCamera,
                                                             const Eigen::Isometry3d& Twc,
                                                             typename PointCloudT::Ptr pCloudWorld)
{
// NOTE: these pcl functions do not copy all the custom fields 
// #if !USE_NORMALS
//     pcl::transformPointCloud(*pCloudCamera, *pCloudWorld, Twc.matrix());
// #else
//     pcl::transformPointCloudWithNormals(*pCloudCamera, *pCloudWorld, Twc.matrix());
// #endif

#if 0
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3d, double>(*pCloudCamera, *pCloudWorld, Twc);
#else 
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3f, float>(*pCloudCamera, *pCloudWorld, Twc.cast<float>());
#endif 
}

template<typename PointT>
void PointCloudMap<PointT>::TransformCameraCloudInWorldFrame(const PointCloudT& cloudCamera, 
                                                             const Eigen::Isometry3d& Twc, 
                                                             PointCloudT& cloudWorld)
{
// NOTE: these pcl functions do not copy all the custom fields 
// #if !USE_NORMALS
//     pcl::transformPointCloud(cloudCamera, cloudWorld, Twc.matrix());
// #else
//     pcl::transformPointCloudWithNormals(cloudCamera, cloudWorld, Twc.matrix());
// #endif    

#if 0
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3d, double>(cloudCamera, cloudWorld, Twc);
#else 
    PointCloudUtils::transformCloud<PointT, PointT, Eigen::Isometry3f, float>(cloudCamera, cloudWorld, Twc.cast<float>());
#endif 
}

template<typename PointT>
void PointCloudMap<PointT>::SaveMap(const std::string& filename)
{
    PointCloudMap<PointT>::PointCloudT out_cloud;

    {
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);

    // if there are no vertices, done!
    if (!pPointCloud_) return;
    if (pPointCloud_->empty()) return;

    // invert back RGB colors (we used BGR in PointCloudMapping)
    out_cloud.resize(pPointCloud_->size());
    for(size_t jj=0, jjEnd=pPointCloud_->size(); jj < jjEnd; jj++)
    {
        //out_cloud[jj].x = pPointCloud_->points[jj].x;
        //out_cloud[jj].y = pPointCloud_->points[jj].y;
        //out_cloud[jj].z = pPointCloud_->points[jj].z;

        out_cloud[jj] = pPointCloud_->points[jj];

        out_cloud[jj].r = pPointCloud_->points[jj].b;
        out_cloud[jj].g = pPointCloud_->points[jj].g;
        out_cloud[jj].b = pPointCloud_->points[jj].r;
        out_cloud[jj].a = pPointCloud_->points[jj].a;
    }
    }

    pcl::io::savePLYFileBinary(filename, out_cloud);

    std::cout << "PointCloudMap::SaveMap(): saving done " << std::endl;
}

// --

template <class PointT, typename std::enable_if<!pcl::traits::has_field<PointT, pcl::fields::kfid>::value>::type* = nullptr>
static void writeCustomData(std::fstream& file, const PointT& point, const bool binary)
{
}

template <class PointT, typename std::enable_if<pcl::traits::has_field<PointT, pcl::fields::kfid>::value>::type* = nullptr>
static void writeCustomData(std::fstream& file, const PointT& point, const bool binary)
{        
    if (binary)
    {                 
        file.write((char*) (&(point.label)), sizeof (uint32_t));
        file.write((char*) (&(point.kfid)), sizeof (uint32_t));            
    }
    else file << "\n" << point.label << " " << point.kfid;      
}

// --

template<typename PointT>
bool PointCloudMap<PointT>::WritePLY(PointCloudT& cloud, std::string filename, bool isMesh, bool binary)
{
    std::fstream file;
    file.open(filename.c_str(), std::ios::out | std::ios::binary);
    if (!file.is_open())
    {
        fprintf(stderr, "\nERROR: Could not open File %s for writing!", filename.c_str());
        return false;
    }

    std::vector<PointT, Eigen::aligned_allocator<PointT> >& vertices = cloud.points;

    std::vector<uint32_t> faces;    
    const int verticesPerFace = 3; // a face is defined by a group of three adjacent vertices      
    const int facesSize = vertices.size();
    if(isMesh)
    {
        faces.resize(facesSize);
        for(size_t ii=0, iiEnd=faces.size(); ii<iiEnd; ii++) 
        {
            faces[ii]=ii; // a face is defined by a group of three adjacent vertices 
        }
    }
    
    file << "ply";
    if (binary)file << "\nformat binary_little_endian 1.0";
    else file << "\nformat ascii 1.0";
    file << "\nelement vertex " << vertices.size();
    file << "\nproperty float32 x\nproperty float32 y\nproperty float32 z";
    //if (colors.size())
    file << "\nproperty uchar red\nproperty uchar green\nproperty uchar blue";
#if USE_NORMALS
    file << "\nproperty float32 normal_x\nproperty float32 normal_y\nproperty float32 normal_z";    
#endif        
#if USE_POINTSURFELSEGMENT
    file << "\nproperty uint32 label";    
    file << "\nproperty uint32 kfid";      
#endif    
    if (faces.size())
    {
        file << "\nelement face " << faces.size() / verticesPerFace;
        file << "\nproperty list uint8 int32 vertex_indices";
    }
    /*if (edges.size())
    {
        file << "\nElement edge " << edges.size() / 2;
        file << "\nproperty int vertex1\nproperty int vertex2";
    }*/
    file << "\nend_header";
    if (binary) file << "\n";

    for (size_t i =0, iEnd=vertices.size(); i < iEnd; i++)
    {
        if (binary)
        {
            file.write((char*) (&(vertices[i].x)), 3*sizeof (float));
        }
        else file << "\n" << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z;
        //if (colors.size())
        {
            if (binary)
            {
                file.write((char*) (&(vertices[i].rgba)), 3* sizeof (uint8_t));
            }
            else file << " " << (int) (vertices[i].r) << " " << (int) (vertices[i].g) << " " << (int) (vertices[i].b);
        }
#if USE_NORMALS
        if (binary)
        {
            file.write((char*) (&(vertices[i].normal_x)), 3*sizeof (float));
        }
        else file << "\n" << vertices[i].normal_x << " " << vertices[i].normal_y << " " << vertices[i].normal_z;            
#endif  
#if USE_POINTSURFELSEGMENT
        writeCustomData(file, vertices[i], binary);
#endif          
    }
    for (size_t i = 0, iEnd=faces.size(); i < iEnd; i += verticesPerFace)
    {
        if (binary)
        {
            file.write((char*) (&verticesPerFace), sizeof(uint8_t));
        }
        else file << "\n" << (int) verticesPerFace;
        for (unsigned int j = 0; j < verticesPerFace; j++)
        {
            if (binary)
            {
                unsigned int idx = faces[i + j];
                file.write((char*) (&idx), sizeof (unsigned int));
            }
            else file << " " << (faces[i + j]);
        }
    }
    /*for (unsigned int i = 0; i < edges.size(); i += 2)
    {
        if (binary)
        {
            unsigned int idx = edges[i];
            file.write((char*) (&idx), sizeof (unsigned int));
            idx = edges[i + 1];
            file.write((char*) (&idx), sizeof (unsigned int));
        }
        else file << "\n " << edges[i] << " " << edges[i + 1];
    }*/

    file.close();
    return true;
}


template<typename PointT>
void PointCloudMap<PointT>::SaveTriangleMeshMap(const std::string& filename, bool binary)
{
    PointCloudMap<PointT>::PointCloudT out_cloud;
    {    
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);

    // if there are no vertices, done!
    if (!pPointCloud_) return;
    if (pPointCloud_->empty()) return;

    // invert back RGB colors (we used BGR in PointCloudMapping)
    out_cloud.resize(pPointCloud_->size());
    for (size_t jj = 0, jjEnd=pPointCloud_->size(); jj < jjEnd; jj++)
    {
        out_cloud[jj] = pPointCloud_->points[jj];

        out_cloud[jj].r = pPointCloud_->points[jj].r;
        out_cloud[jj].g = pPointCloud_->points[jj].g;
        out_cloud[jj].b = pPointCloud_->points[jj].b;
        out_cloud[jj].a = pPointCloud_->points[jj].a;
    }
    }
    
    this->WritePLY(out_cloud, filename, true);

    std::cout << "PointCloudMap::SaveMap(): saving done " << std::endl;
}

template<typename PointT>
bool PointCloudMap<PointT>::LoadMap(const std::string& filename)
{
    std::unique_lock<std::recursive_timed_mutex> lck(pointCloudMutex_);
    
    std::cout << "PointCloudMap::LoadMap(): " << filename << std::endl;     
    {
    if (!Utils::fileExist(filename)) 
    {
        std::cerr << "Cannot open dense map file: " << filename << std::endl;
        return false;
    }
    }    
    
    //pcl::PCLPointCloud2 point_cloud2; 
    pcl::PolygonMesh mesh;
    
    pcl::io::loadPLYFile(filename, mesh);
    
    //pcl::fromPCLPointCloud2( point_cloud2, *pPointCloud_);  
    pcl::fromPCLPointCloud2(mesh.cloud, *pPointCloud_); 
    
    std::cout << "number of points: " << pPointCloud_->size() << std::endl; 
    
    bool bHasNormals = false;
    for(size_t ii=0; ii < mesh.cloud.fields.size(); ii++)
    {
        const pcl::PCLPointField& field = mesh.cloud.fields[ii];
        if( field.name == "normal_x") bHasNormals = true; 
        //std::cout << "read field: " << field << std::endl; 
    }
    
    if(!bHasNormals)
    {
        // estimate normals 
        this->ComputeNormals(pPointCloud_);
    }
    
    // invert back RGB colors (we used BGR in PointCloudMapping)
    this->InvertColors(pPointCloud_);

    return true;
}

template<typename PointT>
void PointCloudMap<PointT>::InvertColors(typename PointCloudT::Ptr pCloud)
{        
    if (pCloud->empty()) return;
    
    // invert back RGB colors (we used BGR in PointCloudMapping)
    for (size_t jj = 0, jjEnd=pCloud->size(); jj < jjEnd; jj++)
    {
        std::swap(pCloud->points[jj].r, pCloud->points[jj].b); 
    }    
}

template<typename PointT>
void PointCloudMap<PointT>::ComputeNormals(typename PointCloudT::Ptr pCloud)
{
    
#if USE_NORMALS
    
    // run-time check if we have a cloud with normals 
    if( pcl::traits::has_field<PointT, pcl::fields::normal_x>::value )
    {            
        std::cout << "estimating normals ... " << std::endl; 

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setInputCloud(pCloud);
        
        // < trying to ensure the normals are all pointed in the same direction 
        //ne.setViewPoint(0,0,0);
        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typename pcl::search::KdTree<PointT>::Ptr pTree (new typename pcl::search::KdTree<PointT> ());
        ne.setSearchMethod (pTree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr pCloudNormals (new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 
        //ne.setRadiusSearch (3*resolution_);
        ne.setKSearch(26); // ideally a 26-connected neighborhood 

        // Compute the features
        ne.compute (*pCloudNormals);

        for(size_t ii=0, iEnd=pCloud->size(); ii<iEnd; ii++)
        {
            pCloud->points[ii].normal_x = pCloudNormals->points[ii].normal_x;
            pCloud->points[ii].normal_y = pCloudNormals->points[ii].normal_y;
            pCloud->points[ii].normal_z = pCloudNormals->points[ii].normal_z;
        }

        std::cout << "estimating normals ... done" << std::endl;     
    }

#endif
    
}

} //namespace PLVS2