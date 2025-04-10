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

#include "PointCloudMapOctreePointCloud.h"

#include "PointUtils.h"

#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/octree/octree_search.h>

#include <chisel_server/ChiselServer.h>
#include <chisel_server/Conversions.h>


#include "TimeUtils.h"
#include "Converter.h"
#include "Utils.h"
#include "Stopwatch.h"
#include "KeyFrame.h"


#define BUILD_UNSTABLE_AS_CARVED

#define USE_OCTREE_SEARCH 0   

namespace PLVS2
{


template<typename PointT>
const int PointCloudMapOctreePointCloud<PointT>::kDefaultPointCounterThreshold = 3;

template<typename PointT>
const int PointCloudMapOctreePointCloud<PointT>::kFactorCarvingThreshold = 6; // carving_threshold_= kFactorCarvingThreshold*resolution_in; 

template<typename PointT>
const int PointCloudMapOctreePointCloud<PointT>::kFactorSigmaZ=6; 

template<typename PointT>
const float PointCloudMapOctreePointCloud<PointT>::kMinCarvingThreshold = 0.05f; // [m]

template<typename PointT>
const std::uint64_t PointCloudMapOctreePointCloud<PointT>::kDeltaTimeForCleaningUnstablePointsUs = 10 * 1e6; // [microseconds]

template<typename PointT>
PointCloudMapOctreePointCloud<PointT>::PointCloudMapOctreePointCloud(Map* pMap, const std::shared_ptr<PointCloudMapParameters>& params) : PointCloudMap<PointT>(pMap, params), octree_(params->resolution)
{
    //this->bPerformCarving_ = useCarving_in;

    //carvingThreshold_ = std::min(float(kFactorCarvingThreshold * sqrt(3.0f) * resolution_in), kMinCarvingThreshold);
    carvingThreshold_ = std::min(float(kFactorCarvingThreshold * params->resolution), kMinCarvingThreshold);

    voxelFilter_.setLeafSize(params->resolution, params->resolution, params->resolution);
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::SetDepthCameraModel(const PointCloudCamParams& params)
{
    depthCameraModel_ = chisel_server::ToChiselCamera(params.fx, params.fy, params.cx, params.cy, params.width, params.height);
    depthCameraModel_.SetNearPlane(params.minDist);
    depthCameraModel_.SetFarPlane(params.maxDist);
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::InsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_world)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    octree_.setInputCloud(cloud_world); 
    octree_.addPointsFromInputCloud(); /// TODO: add more efficient insertion in case of no segmentation (namely, avoid checking the new label of each point)
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::ReinsertCloud(typename PointCloudMap<PointT>::PointCloudT::ConstPtr cloud_world)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    octree_.setInputCloud(cloud_world); 
    octree_.reinsertPointsFromInputCloud(); /// TODO: add more efficient insertion in case of no segmentation (namely, avoid checking the new label of each point)
}

//template<typename PointT>
//void PointCloudMapOctreePointCloud<PointT>::InsertCloudWithDepthOld(typename PointCloudT::ConstPtr cloud_world, const cv::Mat& Twc, const cv::Mat& depthImage, double max_range)
//{
//    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);
//
//    if (this->bPerformCarving_ && !depthImage.empty() && this->pPointCloud_)
//    {
//
//        if (this->bMapUpdated_)
//        {
//            this->bMapUpdated_ = false;
//            if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
//#ifdef BUILD_UNSTABLE_AS_CARVED                
//            this->pPointCloudUnstable_->clear();
//#endif
//        }
//
//#if 0        
//        cv::Mat depthImageFiltered;
//        cv::GaussianBlur(depthImage, depthImageFiltered, cv::Size(5, 5), 5, 5);
//#else
//        const cv::Mat& depthImageFiltered = depthImage;
//#endif
//
//        int num_removed_points = 0;
//
//        chisel::Transform transform(chisel::Transform::Identity());
//        transform.translation()(0) = Twc.at<float>(0, 3);
//        transform.translation()(1) = Twc.at<float>(1, 3);
//        transform.translation()(2) = Twc.at<float>(2, 3);
//        transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
//
//        chisel::Frustum frustum;
//        chisel::AABB box;
//        depthCameraModel_.SetupFrustum(transform, &frustum);
//        frustum.ComputeBoundingBox(&box);
//
//        // 1. crop point cloud with AABB of frustrum 
//
//        Eigen::Vector3f& point_box_min = box.min;
//        Eigen::Vector3f& point_box_max = box.max;
//        PointCloudT point_cloud_in_frustrum;
//
//#if 1        
//        /// < TODO replace with index filtering 
//        /// < search by using preliminary crop box filter
//        pcl::CropBox<PointT> cropbox_pcl;
//        cropbox_pcl.setMin(Eigen::Vector4f(point_box_min[0], point_box_min[1], point_box_min[2], 1.));
//        cropbox_pcl.setMax(Eigen::Vector4f(point_box_max[0], point_box_max[1], point_box_max[2], 1.));
//        cropbox_pcl.setInputCloud(this->pPointCloud_);
//        cropbox_pcl.filter(point_cloud_in_frustrum);
//#else
//        /// < search by using octree structure  NOTE: this is slower given the octree recursive structure!!
//        octree_.boxSearch(point_box_min, point_box_max, point_cloud_in_frustrum);
//#endif
//
//        // 2. check if point is in image and project it in index map (each pixel contains a list of tuples <point index, depth > with depth < sensed depth)
//        cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
//        cv::Mat twc = Twc.rowRange(0, 3).col(3);
//        cv::Mat Rcw = Rwc.t();
//        cv::Mat tcw = -Rcw*twc;
//        const float fx = depthCameraModel_.GetIntrinsics().GetFx();
//        const float fy = depthCameraModel_.GetIntrinsics().GetFy();
//        const float cx = depthCameraModel_.GetIntrinsics().GetCx();
//        const float cy = depthCameraModel_.GetIntrinsics().GetCy();
//        const int width = depthCameraModel_.GetWidth();
//        const int height = depthCameraModel_.GetHeight();
//        for (size_t ii = 0; ii < point_cloud_in_frustrum.size(); ii++)
//        {
//            const cv::Mat P = (cv::Mat_<float>(3, 1) << point_cloud_in_frustrum[ii].x, point_cloud_in_frustrum[ii].y, point_cloud_in_frustrum[ii].z);
//
//            // 3D in camera coordinates
//            const cv::Mat Pc = Rcw * P + tcw;
//            const float &PcX = Pc.at<float>(0);
//            const float &PcY = Pc.at<float>(1);
//            const float &PcZ = Pc.at<float>(2);
//
//            // Check positive depth
//            if (PcZ < 0.0f) continue;
//
//            // Project in image and check it is not outside
//            const float invz = 1.0f / PcZ;
//            const float u = fx * PcX * invz + cx;
//            const float v = fy * PcY * invz + cy;
//
//            const int ud = round(u);
//            if ((ud < 0) || (ud >= width)) continue;
//            const int vd = round(v);
//            if ((vd < 0) || (vd >= height)) continue;
//
//            //std::cout << "point img : " << ud << ", " << vd << "img size: " << depthImage.size() << std::endl;
//
//            /// < TODO: interpolate close values ?
//            const float d = depthImageFiltered.at<float>(vd, ud);
//            if ((std::isnan(d)) || (d >= max_range)) continue;
//            if (PcZ < (d - carvingThreshold_))
//            {
//
//#ifdef BUILD_UNSTABLE_AS_CARVED
//                this->pPointCloudUnstable_->push_back(point_cloud_in_frustrum[ii]);
//#endif
//
//                //octree_.deleteVoxelAtPoint(point_cloud_in_frustrum[ii]);                
//                typename OctreeType::LeafContainer* leaf = octree_.findLeafAtPointPublic(point_cloud_in_frustrum[ii]);
//                if (leaf) leaf->reset();
//
//                num_removed_points++;
//            }
//            else
//            {
//
//            }
//        }
//        std::cout << "using carving - removed " << num_removed_points << " points" << std::endl;
//
//    }
//
//    octree_.setInputCloud(cloud_world);
//    octree_.addPointsFromInputCloud();
//}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::InsertCloudWithDepthOld(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    typename PointCloudT::Ptr cloud_camera = pData->pPointCloudKeyFrame->pCloudCamera;
    const cv::Mat& depthImage = pData->pPointCloudKeyFrame->imgDepth;
    const cv::Mat& pixelToPointIndex = pData->pPointCloudKeyFrame->imgPointIndex;
    LabelMap& labelMap = pData->pPointCloudKeyFrame->labelMap;
    const double max_range = pData->maxRange;

    TICKCLOUD("seg-cardinality");
    if (this->pPointCloudMapParameters_->bSegmentationOn)
    {
        std::vector<unsigned int>& scanPcLabelsCardinality = labelMap.GetScanPCLabelsCardinality();
        if (scanPcLabelsCardinality.empty())
        {
            typename PointCloudMap<PointT>::PointCloudT::Ptr filteredCameraCloud(new PointCloudT());
            this->voxelFilter_.setInputCloud(pData->pPointCloudKeyFrame->pCloudCamera);
            this->voxelFilter_.filter(*filteredCameraCloud);
            PointUtils::computeScanPcLabeCardinality<PointT>(filteredCameraCloud, labelMap.GetNumLabels(), scanPcLabelsCardinality);
        }
    }
    TOCKCLOUD("seg-cardinality");

    //int count = 0;

    if ((this->pPointCloudMapParameters_->bUseCarving || this->pPointCloudMapParameters_->bSegmentationOn) && !depthImage.empty() && this->pPointCloud_)
    {

        if (this->bMapUpdated_)
        {
            this->bMapUpdated_ = false;
            if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
#ifdef BUILD_UNSTABLE_AS_CARVED                
            this->pPointCloudUnstable_->clear();
#endif
        }

        int num_removed_points = 0;

        chisel::Transform transform(chisel::Transform::Identity());
        // transform.translation()(0) = Twc.at<float>(0, 3);
        // transform.translation()(1) = Twc.at<float>(1, 3);
        // transform.translation()(2) = Twc.at<float>(2, 3);
        // transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
        transform.translation() = Twc.translation();
        transform.linear() = Twc.rotationMatrix();

        chisel::Frustum frustum;
        chisel::AABB box;
        depthCameraModel_.SetupFrustum(transform, &frustum);
        frustum.ComputeBoundingBox(&box);

        // 1. crop point cloud with AABB of frustrum 

        const Eigen::Vector3f& point_box_min = box.min;
        const Eigen::Vector3f& point_box_max = box.max;


        TICKCLOUD("octreepointPA");

     
#if !USE_OCTREE_SEARCH        

        /// < search by using preliminary crop box filter
        pcl::CropBox<PointT> cropbox_pcl;
        cropbox_pcl.setMin(Eigen::Vector4f(point_box_min[0], point_box_min[1], point_box_min[2], 1.));
        cropbox_pcl.setMax(Eigen::Vector4f(point_box_max[0], point_box_max[1], point_box_max[2], 1.));
        cropbox_pcl.setInputCloud(this->pPointCloud_);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(this->pPointCloud_);
        pcl::PointIndices::Ptr pFilteredIndices(new pcl::PointIndices());

        cropbox_pcl.filter(pFilteredIndices->indices); // filter and write the output in indices_filtered->indices
        extract.setIndices(pFilteredIndices);
#else
        /// < search by using octree structure  NOTE: this results slower for medium size clouds (conside the octree recursive structure), it could be faster for very large cloud 
        PointCloudT point_cloud_in_frustrum;
        octree_.boxSearch(point_box_min, point_box_max, point_cloud_in_frustrum);
#endif

        // 2. check if point is in image and project it in index map (each pixel contains a list of tuples <point index, depth > with depth < sensed depth)
        // cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        // cv::Mat twc = Twc.rowRange(0, 3).col(3);
        // cv::Mat Rcw = Rwc.t();
        // cv::Mat tcw = -Rcw*twc;
        const Sophus::SE3f Tcw = Twc.inverse();
        const Sophus::Matrix3f Rcw = Tcw.rotationMatrix(); 
        const Sophus::Vector3f tcw = Tcw.translation();         
        const float fx = depthCameraModel_.GetIntrinsics().GetFx();
        const float fy = depthCameraModel_.GetIntrinsics().GetFy();
        const float cx = depthCameraModel_.GetIntrinsics().GetCx();
        const float cy = depthCameraModel_.GetIntrinsics().GetCy();
        const int width = depthCameraModel_.GetWidth();
        const int height = depthCameraModel_.GetHeight();
        
        int pixelToPointIndexColsMin1 = pixelToPointIndex.cols-1;
        int pixelToPointIndexRowsMin1 = pixelToPointIndex.rows-1;

#if !USE_OCTREE_SEARCH            
        for (size_t ii = 0, iiEnd=pFilteredIndices->indices.size(); ii < iiEnd; ii++)
        {
            const int index = pFilteredIndices->indices[ii];
            const PointT& mapPointW = this->pPointCloud_->points[index];
#else
        for (size_t ii = 0, iiEnd=point_cloud_in_frustrum.size(); ii < iiEnd; ii++)
        {
            const PointT& mapPointW = point_cloud_in_frustrum[ii];
#endif 

            PointT mapPointC;
            PointUtils::transformPoint(mapPointW, Rcw, tcw, mapPointC);
            const float &PcX = mapPointC.x;
            const float &PcY = mapPointC.y;
            const float &PcZ = mapPointC.z;

            // Check positive depth
            if (PcZ < 0.0f) continue;

            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = fx * PcX * invz + cx;
            const float v = fy * PcY * invz + cy;

            const int ui = lrint(u);
            if ((ui < 0) || (ui >= width)) continue;
            const int vi = lrint(v);
            if ((vi < 0) || (vi >= height)) continue;

            //std::cout << "point img : " << ud << ", " << vd << "img size: " << depthImage.size() << std::endl;

            /// < TODO: interpolate close values ?
            const float d = depthImage.at<float>(vi, ui);
            if ((std::isnan(d)) || (d > max_range)) continue;

            const float normP = sqrt(PcX * PcX + PcY * PcY + PcZ * PcZ);
            const float sd = normP * (d * invz - 1.0f); // signed distance if > 0 points are closer than the depth

            const float sixSigmaZ = kFactorSigmaZ * Utils::SigmaZ(d);
            const float distanceTh = (carvingThreshold_ + sixSigmaZ);

            //PointUtils::printDataForLabel(mapPointW,139,count,sd);

            //if (PcZ < (d - (carvingThreshold_+6*Utils::SigmaZ(d)) ) )
            if (sd > distanceTh)
            {

                if (!this->pPointCloudMapParameters_->bUseCarving) continue;

#ifdef BUILD_UNSTABLE_AS_CARVED
                this->pPointCloudUnstable_->push_back(mapPointW);
#endif 
                //octree_.deleteVoxelAtPoint(point_cloud_in_frustrum[ii]);                
                typename OctreeType::LeafContainer* leaf = octree_.findLeafAtPointPublic(mapPointW);
                if (leaf) leaf->reset();

                num_removed_points++;
            }
            else
            {
#if COMPUTE_SEGMENTS                
                if (!this->pPointCloudMapParameters_->bSegmentationOn) continue;
                if (labelMap.IsAlreadyProcessed()) continue;

                if (fabs(sd) < distanceTh)
                {
                    //this->pPointCloudUnstable_->push_back(mapPointW);

                    if (!PointUtils::isValidLabel(mapPointC)) continue;

                    // retrieve the closest downsampled representation 
                    const int ud = std::min((int)lrint(u / this->pPointCloudMapParameters_->nDownsampleStep)*(this->pPointCloudMapParameters_->nDownsampleStep), pixelToPointIndexColsMin1);
                    const int vd = std::min((int)lrint(v / this->pPointCloudMapParameters_->nDownsampleStep)*(this->pPointCloudMapParameters_->nDownsampleStep), pixelToPointIndexRowsMin1);

                    const int scanIndex = pixelToPointIndex.at<int>(vd, ud);
                    if (!(scanIndex < 0)) // check if point index is valid 
                    {
                        const PointT& scanPointC = cloud_camera->points[scanIndex];

                        if (!PointUtils::isValidLabel(scanPointC)) continue;

                        const float scalarProd = PointUtils::normalsDotProduct(mapPointC, scanPointC);
                        if (scalarProd > this->pPointCloudMapParameters_->segmentationMinCosForNormalAssociation)
                        {
                            PointUtils::updateLabelMap(labelMap, mapPointC, scanPointC);
                            //std::cout << "associating map point: " << mapPointC << " with scan point: " << scanPointC << std::endl; 
                            //std::cout << "distance: " << PointUtils::distance(mapPointC,scanPointC) << std::endl; 
                            //std::cout << "normals angle: " << PointUtils::normalsAngle(mapPointC,scanPointC)*180/M_PI << std::endl; 
                        }
                    }


                }

#endif
            }

        } // end for (size_t ii = 0; ii < pFilteredIndices->indices.size(); ii++)

        TOCKCLOUD("octreepointPA");

        if (this->pPointCloudMapParameters_->bUseCarving)
            std::cout << "using carving - removed " << num_removed_points << " points" << std::endl;

#if COMPUTE_SEGMENTS            
        if (this->pPointCloudMapParameters_->bSegmentationOn)
        {
            bool bComputeBestMatches = labelMap.ComputeBestMatches();
            if (bComputeBestMatches)
            {
                //labelMap.PrintMatches();

                PointUtils::updateCloudFromLabelMap<PointT>(labelMap, cloud_world, cloud_camera); /// < TODO ? include cloud_camera which is stored in the pointcloud keyframe

                GlobalLabelMap::GetMap().AddNumLabels(labelMap.GetNumLabels());
                GlobalLabelMap::GetMap().UpdateAll();
                GlobalLabelMap::GetMap().PrintMatches();
            }

            //PointUtils::resetIsolatedLabels<PointT>(cloud_world, 9, 4);
        }
#endif

    }

    octree_.setInputCloud(cloud_world);
    octree_.addPointsFromInputCloud();
}
    


template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::InsertCloudWithDepthNoSegm(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    typename PointCloudT::Ptr cloud_camera = pData->pPointCloudKeyFrame->pCloudCamera;
    const cv::Mat& depthImage = pData->pPointCloudKeyFrame->imgDepth;
    const double min_range = pData->minRange;
    const double max_range = pData->maxRange;

    // TODO: this must be replaced by using Active-Inactive models 
    if ( this->pPointCloudMapParameters_->bUseCarving && !depthImage.empty() && this->pPointCloud_)
    {

        if (this->bMapUpdated_)
        {
            this->bMapUpdated_ = false;
            if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
#ifdef BUILD_UNSTABLE_AS_CARVED                
            this->pPointCloudUnstable_->clear();
#endif
        }

        int num_removed_points = 0;

        Eigen::Affine3f transform(Eigen::Affine3f::Identity());
        // transform.translation()(0) = Twc.at<float>(0, 3);
        // transform.translation()(1) = Twc.at<float>(1, 3);
        // transform.translation()(2) = Twc.at<float>(2, 3);
        // transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
        transform.translation() = Twc.translation();
        transform.linear() = Twc.rotationMatrix();

        Eigen::Matrix4f cam2robot;
        cam2robot << 0, 0, 1, 0,
                     0,-1, 0, 0,
                     1, 0, 0, 0,
                     0, 0, 0, 1;
        Eigen::Matrix4f pose = transform.matrix() * cam2robot;        
        
        const float fx = depthCameraModel_.GetIntrinsics().GetFx();
        const float fy = depthCameraModel_.GetIntrinsics().GetFy();
        const float cx = depthCameraModel_.GetIntrinsics().GetCx();
        const float cy = depthCameraModel_.GetIntrinsics().GetCy();
        const int width = depthCameraModel_.GetWidth();
        const int height = depthCameraModel_.GetHeight();
        
        TICKCLOUD("octreepointPA");

        pcl::FrustumCulling<PointT> fc;
        const float hfov = 2* atan( std::max(cx, width  - cx) / fx );
        const float vfov = 2* atan( std::max(cy, height  - cy) / fy );        
        fc.setVerticalFOV (vfov*180/M_PI);
        fc.setHorizontalFOV (hfov*180/M_PI);
        fc.setNearPlaneDistance (min_range);
        fc.setFarPlaneDistance (max_range);
        fc.setCameraPose (pose);
        fc.setInputCloud(this->pPointCloud_);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(this->pPointCloud_);
        pcl::PointIndices::Ptr pFilteredIndices(new pcl::PointIndices());

        fc.filter(pFilteredIndices->indices); // filter and write the output in indices_filtered->indices
        extract.setIndices(pFilteredIndices);
        

        // 2. check if point is in image and project it in index map (each pixel contains a list of tuples <point index, depth > with depth < sensed depth)
        // const cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        // const cv::Mat twc = Twc.rowRange(0, 3).col(3);
        // const cv::Mat Rcw = Rwc.t();
        // const cv::Mat tcw = -Rcw*twc;
        const Sophus::SE3f Tcw = Twc.inverse();
        const Eigen::Matrix3f Rcw = Tcw.rotationMatrix(); 
        const Eigen::Vector3f tcw = Tcw.translation(); 

        for (size_t ii = 0, iiEnd=pFilteredIndices->indices.size(); ii < iiEnd; ii++)
        {
            const int index = pFilteredIndices->indices[ii];
            const PointT& mapPointW = this->pPointCloud_->points[index];

            PointT mapPointC;
            PointUtils::transformPoint(mapPointW, Rcw, tcw, mapPointC);
            const float &PcX = mapPointC.x;
            const float &PcY = mapPointC.y;
            const float &PcZ = mapPointC.z;

            // Check positive depth
            if (PcZ < 0.0f) continue;

            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = fx * PcX * invz + cx;
            const float v = fy * PcY * invz + cy;

            const int ui = lrint(u);
            if ((ui < 0) || (ui >= width)) continue;
            const int vi = lrint(v);
            if ((vi < 0) || (vi >= height)) continue;

            //std::cout << "point img : " << ud << ", " << vd << "img size: " << depthImage.size() << std::endl;

            /// < TODO: interpolate close values ?
            const float d = depthImage.at<float>(vi, ui);
            if ((std::isnan(d)) || (d > max_range)) continue;

            const float normP = sqrt(PcX * PcX + PcY * PcY + PcZ * PcZ);
            const float sd = normP * (d * invz - 1.0f); // signed distance if > 0 points are closer than the depth

            const float sixSigmaZ = kFactorSigmaZ * Utils::SigmaZ(d);
            const float distanceTh = (carvingThreshold_ + sixSigmaZ);

            //PointUtils::printDataForLabel(mapPointW,139,count,sd);

            //if (PcZ < (d - (carvingThreshold_+6*Utils::SigmaZ(d)) ) )
            if (sd > distanceTh)
            {
#ifdef BUILD_UNSTABLE_AS_CARVED
                this->pPointCloudUnstable_->push_back(mapPointW);
#endif 
                //octree_.deleteVoxelAtPoint(point_cloud_in_frustrum[ii]);                
                typename OctreeType::LeafContainer* leaf = octree_.findLeafAtPointPublic(mapPointW);
                if (leaf) leaf->reset();

                num_removed_points++;
            }

        } // end for (size_t ii = 0; ii < pFilteredIndices->indices.size(); ii++)

        TOCKCLOUD("octreepointPA");

        if (this->pPointCloudMapParameters_->bUseCarving)
            std::cout << "using carving - removed " << num_removed_points << " points" << std::endl;

    } // end     if ( this->bPerformCarving_ && !depthImage.empty() && this->pPointCloud_)

    octree_.setInputCloud(cloud_world);
    octree_.addPointsFromInputCloud();
}    
    

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::InsertCloudWithDepthSegm(typename PointCloudT::Ptr cloud_world, typename PointCloudMapInput<PointT>::Ptr pData, const Sophus::SE3f& Twc)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    typename PointCloudT::Ptr cloud_camera = pData->pPointCloudKeyFrame->pCloudCamera;
    const cv::Mat& depthImage = pData->pPointCloudKeyFrame->imgDepth;
    const cv::Mat& pixelToPointIndex = pData->pPointCloudKeyFrame->imgPointIndex;
    LabelMap& labelMap = pData->pPointCloudKeyFrame->labelMap;
    const double min_range = pData->minRange;
    const double max_range = pData->maxRange;

    TICKCLOUD("seg-cardinality");
    /// < compute the actual cardinality of each segment when integrated in the volumetric map
    std::vector<unsigned int>& scanPcLabelsCardinality = labelMap.GetScanPCLabelsCardinality();
    if (scanPcLabelsCardinality.empty())
    {
        typename PointCloudMap<PointT>::PointCloudT::Ptr filteredCameraCloud(new PointCloudT());
        this->voxelFilter_.setInputCloud(pData->pPointCloudKeyFrame->pCloudCamera);
        this->voxelFilter_.filter(*filteredCameraCloud);
        PointUtils::computeScanPcLabeCardinality<PointT>(filteredCameraCloud, labelMap.GetNumLabels(), scanPcLabelsCardinality);
    }
    TOCKCLOUD("seg-cardinality");

    //int count = 0;

    if (
        ( this->pPointCloudMapParameters_->bUseCarving || (this->pPointCloudMapParameters_->bSegmentationOn && !labelMap.IsAlreadyProcessed()) ) 
        && !depthImage.empty()
        && this->pPointCloud_
    )
    {

        if (this->bMapUpdated_)
        {
            this->bMapUpdated_ = false;
            if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
#ifdef BUILD_UNSTABLE_AS_CARVED                
            this->pPointCloudUnstable_->clear();
#endif
        }

        int num_removed_points = 0;

        Eigen::Affine3f transform(Eigen::Affine3f::Identity());
        // transform.translation()(0) = Twc.at<float>(0, 3);
        // transform.translation()(1) = Twc.at<float>(1, 3);
        // transform.translation()(2) = Twc.at<float>(2, 3);
        // transform.linear() = Converter::toMatrix3f(Twc.rowRange(0, 3).colRange(0, 3));
        transform.translation() = Twc.translation(); 
        transform.linear() = Twc.rotationMatrix(); 

        Eigen::Matrix4f cam2robot;
        cam2robot << 0, 0, 1, 0,
                     0,-1, 0, 0,
                     1, 0, 0, 0,
                     0, 0, 0, 1;
        Eigen::Matrix4f pose = transform.matrix() * cam2robot;        
        
        const auto& intrinsics = depthCameraModel_.GetIntrinsics();
        const float fx = intrinsics.GetFx();
        const float fy = intrinsics.GetFy();
        const float cx = intrinsics.GetCx();
        const float cy = intrinsics.GetCy();
        
        const int width = depthCameraModel_.GetWidth();
        const int height = depthCameraModel_.GetHeight();
        
        TICKCLOUD("octreepointPA");

        pcl::FrustumCulling<PointT> fc;
        const float hfov = 2* atan( std::max(cx, width  - cx) / fx );
        const float vfov = 2* atan( std::max(cy, height  - cy) / fy );        
        fc.setVerticalFOV (vfov*180/M_PI);
        fc.setHorizontalFOV (hfov*180/M_PI);
        fc.setNearPlaneDistance (min_range);
        fc.setFarPlaneDistance (max_range);
        fc.setCameraPose (pose);
        fc.setInputCloud(this->pPointCloud_);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(this->pPointCloud_);
        pcl::PointIndices::Ptr pFilteredIndices(new pcl::PointIndices());

        fc.filter(pFilteredIndices->indices); // filter and write the output in indices_filtered->indices
        extract.setIndices(pFilteredIndices);
        

        // 2. check if point is in image and project it in index map (each pixel contains a list of tuples <point index, depth > with depth < sensed depth)
        // const cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
        // const cv::Mat twc = Twc.rowRange(0, 3).col(3);
        // const cv::Mat Rcw = Rwc.t();
        // const cv::Mat tcw = -Rcw*twc;
        const Sophus::SE3f Tcw = Twc.inverse();
        const Sophus::Matrix3f Rcw = Tcw.rotationMatrix(); 
        const Sophus::Vector3f tcw = Tcw.translation();    

        const int pixelToPointIndexColsMin1 = pixelToPointIndex.cols-1;
        const int pixelToPointIndexRowsMin1 = pixelToPointIndex.rows-1;
        
        for (size_t ii = 0, iiEnd=pFilteredIndices->indices.size(); ii < iiEnd; ii++)
        {
            const int index = pFilteredIndices->indices[ii];
            const PointT& mapPointW = this->pPointCloud_->points[index];

            PointT mapPointC;
            PointUtils::transformPoint(mapPointW, Rcw, tcw, mapPointC);
            const float &PcX = mapPointC.x;
            const float &PcY = mapPointC.y;
            const float &PcZ = mapPointC.z;

            // Check positive depth
            if (PcZ < 0.0f) continue;

            // Project in image and check it is not outside
            const float invz = 1.0f / PcZ;
            const float u = fx * PcX * invz + cx;
            const float v = fy * PcY * invz + cy;

            const int ui = lrint(u);
            if ((ui < 0) || (ui >= width)) continue;
            const int vi = lrint(v);
            if ((vi < 0) || (vi >= height)) continue;

            //std::cout << "point img : " << ud << ", " << vd << "img size: " << depthImage.size() << std::endl;

            /// < TODO: interpolate close values ?
            const float d = depthImage.at<float>(vi, ui);
            if ((std::isnan(d)) || (d > max_range)) continue;

            const float normP = sqrt(PcX * PcX + PcY * PcY + PcZ * PcZ);
            const float sd = normP * (d * invz - 1.0f); // signed distance if > 0 points are closer than the depth

            const float sixSigmaZ = kFactorSigmaZ * Utils::SigmaZ(d);
            const float distanceTh = (carvingThreshold_ + sixSigmaZ);

            //PointUtils::printDataForLabel(mapPointW,139,count,sd);

            //if (PcZ < (d - (carvingThreshold_+6*Utils::SigmaZ(d)) ) )
            if (sd > distanceTh)
            {
                if (!this->pPointCloudMapParameters_->bUseCarving) continue;

#ifdef BUILD_UNSTABLE_AS_CARVED
                this->pPointCloudUnstable_->push_back(mapPointW);
#endif 
                //octree_.deleteVoxelAtPoint(point_cloud_in_frustrum[ii]);                
                typename OctreeType::LeafContainer* leaf = octree_.findLeafAtPointPublic(mapPointW);
                if (leaf) leaf->reset();

                num_removed_points++;
            }
            else
            {
#if COMPUTE_SEGMENTS                
                if (labelMap.IsAlreadyProcessed()) continue;

                if (fabs(sd) < distanceTh)
                {
                    //this->pPointCloudUnstable_->push_back(mapPointW);

                    if (!PointUtils::isValidLabel(mapPointC)) continue;

                    // retrieve the closest downsampled representation 
                    const int ud = std::min((int)lrint(u / this->pPointCloudMapParameters_->nDownsampleStep)*(this->pPointCloudMapParameters_->nDownsampleStep), pixelToPointIndexColsMin1);
                    const int vd = std::min((int)lrint(v / this->pPointCloudMapParameters_->nDownsampleStep)*(this->pPointCloudMapParameters_->nDownsampleStep), pixelToPointIndexRowsMin1);

                    const int scanIndex = pixelToPointIndex.at<int>(vd, ud);
                    if (!(scanIndex < 0)) // check if point index is valid 
                    {
                        const PointT& scanPointC = cloud_camera->points[scanIndex];

                        if (!PointUtils::isValidLabel(scanPointC)) continue;

                        const float scalarProd = PointUtils::normalsDotProduct(mapPointC, scanPointC);
                        if (scalarProd > this->pPointCloudMapParameters_->segmentationMinCosForNormalAssociation)
                        {
                            PointUtils::updateLabelMap(labelMap, mapPointC, scanPointC);
                            //std::cout << "associating map point: " << mapPointC << " with scan point: " << scanPointC << std::endl; 
                            //std::cout << "distance: " << PointUtils::distance(mapPointC,scanPointC) << std::endl; 
                            //std::cout << "normals angle: " << PointUtils::normalsAngle(mapPointC,scanPointC)*180/M_PI << std::endl; 
                        }
                    }


                }

#endif
            }

        } // end for (size_t ii = 0; ii < pFilteredIndices->indices.size(); ii++)

        TOCKCLOUD("octreepointPA");

        if (this->pPointCloudMapParameters_->bUseCarving)
            std::cout << "using carving - removed " << num_removed_points << " points" << std::endl;

#if COMPUTE_SEGMENTS            
        if (this->pPointCloudMapParameters_->bSegmentationOn)
        {
            bool bComputeBestMatches = labelMap.ComputeBestMatches();
            if (bComputeBestMatches)
            {
                //labelMap.PrintMatches();

                PointUtils::updateCloudFromLabelMap<PointT>(labelMap, cloud_world, cloud_camera); /// < TODO ? include cloud_camera which is stored in the pointcloud keyframe

                GlobalLabelMap::GetMap().AddNumLabels(labelMap.GetNumLabels());
                GlobalLabelMap::GetMap().UpdateAll();
                GlobalLabelMap::GetMap().PrintMatches();
            }

            //PointUtils::resetIsolatedLabels<PointT>(cloud_world, 9, 4);
        }
#endif

    }

    octree_.setInputCloud(cloud_world);
    octree_.addPointsFromInputCloud();
}
    

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::InsertData(typename PointCloudMapInput<PointT>::Ptr pData)
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    this->lastTimestamp_ = pData->timestamp;

    assert(pData->type == PointCloudMapInput<PointT>::kPointCloudAndDepthImage);
    
    KeyFramePtr pKF = pData->pPointCloudKeyFrame->pKF;
    uint32_t kfid = pKF->mnId;    
    this->mapKfidPointCloudKeyFrame_[kfid] = pData->pPointCloudKeyFrame;        

    typename PointCloudT::Ptr pCloudWorld(new PointCloudT);
    Sophus::SE3f Twc = pData->pPointCloudKeyFrame->GetCameraPose();
    pData->pPointCloudKeyFrame->TwcIntegration = Twc; 
    
    this->TransformCameraCloudInWorldFrame(pData->pPointCloudKeyFrame->pCloudCamera, Converter::toIsometry3d(Twc), pCloudWorld);

    switch (pData->type)
    {
    
        case PointCloudMapInput<PointT>::kPointCloudAndDepthImage:
            if(this->pPointCloudMapParameters_->bSegmentationOn)
            {
                this->InsertCloudWithDepthSegm(pCloudWorld, pData, Twc);
            }
            else
            {
                this->InsertCloudWithDepthNoSegm(pCloudWorld, pData, Twc);
            }            
            break;
        
        default:
            this->InsertCloud(pCloudWorld);
    }
}

template<typename PointT>
int PointCloudMapOctreePointCloud<PointT>::UpdateMap()
{
    if(this->pPointCloudMapParameters_->bSegmentationOn)
    {
        return this->UpdateMapSegm();
    }
    else
    {
        if(this->pPointCloudMapParameters_->bRemoveUnstablePoints)
        {
            return this->UpdateMapNoSegm();
        }
        else
        {
            return this->UpdateMapNoSegmNoRemoveUnstable();
        }
    }      
}

template<typename PointT>
int PointCloudMapOctreePointCloud<PointT>::UpdateMapNoSegmNoRemoveUnstable()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    PointCloudMap<PointT>::ResetPointCloud();

#ifndef BUILD_UNSTABLE_AS_CARVED
    if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
    this->pPointCloudUnstable_->clear();
#endif

    //int num_removed_unstable_points = 0;

    /// < updatemap

    //    typename OctreeType::AlignedPointTVector voxel_center_list;
    //    octree_.getOccupiedVoxelCenters(voxel_center_list);
    //    for(size_t ii=0; ii< voxel_center_list.size(); ii++)
    //        this->pPointCloud_->push_back(voxel_center_list[ii]);

    //const int threshold = this->nPointCounterThreshold_ - 1;
    const unsigned int threshold = std::max(this->pPointCloudMapParameters_->nPointCounterThreshold - 1, 0);    
    
    typename OctreeType::LeafNodeIterator it, itEnd;
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10    
    for (it = octree_.leaf_begin(), itEnd = octree_.leaf_end(); it != itEnd; it++)
#else
    for (it = octree_.leaf_depth_begin(), itEnd = octree_.leaf_depth_end(); it != itEnd; ++it)
#endif 
    {
        //if(it.getCurrentOctreeDepth() == octree_.getTreeDepth())
        {
            typename OctreeType::LeafContainer& leaf = it.getLeafContainer();

            //if(octree_.isVoxelOccupiedAtPoint(point))
            if (leaf.getCounter() > threshold)
            {
                PointT& mapPoint = leaf.getCentroid();
           
                this->pPointCloud_->push_back(mapPoint);
            }
        }
    }
    
    //std::cout << "PointCloudMapOctreePointCloud<PointT>::UpdateMap() - removed " << num_removed_unstable_points << " unstable points" << std::endl;

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}


template<typename PointT>
int PointCloudMapOctreePointCloud<PointT>::UpdateMapNoSegm()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    PointCloudMap<PointT>::ResetPointCloud();

#ifndef BUILD_UNSTABLE_AS_CARVED
    if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
    this->pPointCloudUnstable_->clear();
#endif

    int num_removed_unstable_points = 0;

    /// < updatemap

    //    typename OctreeType::AlignedPointTVector voxel_center_list;
    //    octree_.getOccupiedVoxelCenters(voxel_center_list);
    //    for(size_t ii=0; ii< voxel_center_list.size(); ii++)
    //        this->pPointCloud_->push_back(voxel_center_list[ii]);

    //const int threshold = this->nPointCounterThreshold_ - 1;
    const unsigned int threshold = std::max(this->pPointCloudMapParameters_->nPointCounterThreshold - 1, 0);    

    typename OctreeType::LeafNodeIterator it, itEnd;
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10    
    for (it = octree_.leaf_begin(), itEnd = octree_.leaf_end(); it != itEnd; it++)
#else
    for (it = octree_.leaf_depth_begin(), itEnd = octree_.leaf_depth_end(); it != itEnd; ++it)
#endif     
    {
        //if(it.getCurrentOctreeDepth() == octree_.getTreeDepth())
        {
            typename OctreeType::LeafContainer& leaf = it.getLeafContainer();

            //if(octree_.isVoxelOccupiedAtPoint(point))
            if (leaf.getCounter() > threshold)
            {
                PointT& mapPoint = leaf.getCentroid();
           
                this->pPointCloud_->push_back(mapPoint);
            }
            else
            {
                // remove unstable points 
                const PointT& mapPoint = leaf.getCentroid();

#ifndef BUILD_UNSTABLE_AS_CARVED                
                this->pPointCloudUnstable_->push_back(point);
#endif                
                std::uint64_t deltaT = this->lastTimestamp_ - leaf.getTimestamp();
                //std::cout << "deltaT: " << deltaT << "pt: " << leaf.getTimestamp() << ", lt:" << this->lastTimestamp_ << std::endl; 
                // clean old unstable points 
                if (deltaT > kDeltaTimeForCleaningUnstablePointsUs)
                {
                    num_removed_unstable_points++;
                    octree_.deleteVoxelAtPoint(mapPoint);
                }
            }
        }
    }
    
    std::cout << "PointCloudMapOctreePointCloud<PointT>::UpdateMap() - removed " << num_removed_unstable_points << " unstable points" << std::endl;

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}


template<typename PointT>
int PointCloudMapOctreePointCloud<PointT>::UpdateMapSegm()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    PointCloudMap<PointT>::ResetPointCloud();

#ifndef BUILD_UNSTABLE_AS_CARVED
    if (!this->pPointCloudUnstable_) this->pPointCloudUnstable_.reset(new PointCloudT());
    this->pPointCloudUnstable_->clear();
#endif

    int num_removed_unstable_points = 0;

    /// < updatemap

    //    typename OctreeType::AlignedPointTVector voxel_center_list;
    //    octree_.getOccupiedVoxelCenters(voxel_center_list);
    //    for(size_t ii=0; ii< voxel_center_list.size(); ii++)
    //        this->pPointCloud_->push_back(voxel_center_list[ii]);

#if COMPUTE_SEGMENTS      
    GlobalLabelMap::MapLabelAssociations& labelMapForMerging = GlobalLabelMap::GetMap().GetLabelMapForMerging();
    const GlobalLabelMap::LabelType minMapLabelToMerge = GlobalLabelMap::GetMap().GetMinLabelToMerge();
    const GlobalLabelMap::LabelType maxMapLabelToMerge = GlobalLabelMap::GetMap().GetMaxLabelToMerge();
    //GlobalLabelMap::MapLabelCardinality& mapLabelsCardinality = GlobalLabelMap::GetMap().GetMapLabelsCardinality();

    //mapLabelsCardinality.clear();
#endif

    typename OctreeType::LeafNodeIterator it, itEnd;
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10    
    for (it = octree_.leaf_begin(), itEnd = octree_.leaf_end(); it != itEnd; it++)
#else
    for (it = octree_.leaf_depth_begin(), itEnd = octree_.leaf_depth_end(); it != itEnd; ++it)
#endif     
    {
        //if(it.getCurrentOctreeDepth() == octree_.getTreeDepth())
        {
            typename OctreeType::LeafContainer& leaf = it.getLeafContainer();

            //if(octree_.isVoxelOccupiedAtPoint(point))
            if (leaf.getCounter() >= this->pPointCloudMapParameters_->nPointCounterThreshold)
            {
                PointT& mapPoint = leaf.getCentroid();

#if COMPUTE_SEGMENTS                     
                // relabel according to matched map labels 
                PointUtils::updatePointLabelMap(labelMapForMerging, minMapLabelToMerge, maxMapLabelToMerge, mapPoint);
#endif                 
                this->pPointCloud_->push_back(mapPoint);
            }
            else
            {
                // remove unstable points 
                const PointT& mapPoint = leaf.getCentroid();

#ifndef BUILD_UNSTABLE_AS_CARVED                
                this->pPointCloudUnstable_->push_back(point);
#endif                
                std::uint64_t deltaT = this->lastTimestamp_ - leaf.getTimestamp();
                //std::cout << "deltaT: " << deltaT << "pt: " << leaf.getTimestamp() << ", lt:" << this->lastTimestamp_ << std::endl; 
                // clean old unstable points 
                if (deltaT > kDeltaTimeForCleaningUnstablePointsUs)
                {
                    num_removed_unstable_points++;
                    octree_.deleteVoxelAtPoint(mapPoint);
                }
            }
        }
    }
    
    std::cout << "PointCloudMapOctreePointCloud<PointT>::UpdateMap() - removed " << num_removed_unstable_points << " unstable points" << std::endl;

    /// < update timestamp !
    this->UpdateMapTimestamp();

    return this->pPointCloud_->size();
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::Clear()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    octree_ = OctreeType(this->pPointCloudMapParameters_->resolution);

    /// < clear basic class !
    PointCloudMap<PointT>::Clear();
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::OnMapChange()
{
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapOctreePointCloud<PointT>::OnMapChange() - octree reset *** " << std::endl;
        octree_ = OctreeType(this->pPointCloudMapParameters_->resolution);
    }
}
    
template<>
void PointCloudMapOctreePointCloud<pcl::PointSurfelSegment>::OnMapChange()
{
    typedef pcl::PointSurfelSegment PointT;
    typedef typename PointCloudMap<PointT>::PointCloudT PointCloudT;
    typedef std::unordered_map<uint32_t, typename PointCloudMap<PointT>::PointCloudT> MapKfidCloud;
    
    std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

    if (this->pPointCloudMapParameters_->bResetOnSparseMapChange)
    {
        std::cout << "PointCloudMapOctreePointCloud<PointT>::OnMapChange() - octree reset *** " << std::endl;
        octree_ = OctreeType(this->pPointCloudMapParameters_->resolution);
    }  
    
    if(this->pPointCloudMapParameters_->bCloudDeformationOnSparseMapChange)
    {
        std::cout << "PointCloudMapVoxelGridFilterActive<PointT>::OnMapChange() - point cloud KF adjustment" << std::endl;
        
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
        octree_ = OctreeType(this->pPointCloudMapParameters_->resolution);

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

            // cv::Mat TcwIntegration = cv::Mat::eye(4,4,CV_32F);
            // // invert by taking into account the structure of the homogeneous transformation matrix
            // cv::Mat RcwIntegration =  TwcIntegration.rowRange(0,3).colRange(0,3).t();
            // cv::Mat tcwIntegration = -RcwIntegration*TwcIntegration.rowRange(0,3).col(3);
            // RcwIntegration.copyTo(TcwIntegration.rowRange(0,3).colRange(0,3));
            // tcwIntegration.copyTo(TcwIntegration.rowRange(0,3).col(3));
            Sophus::SE3f TcwIntegration = TwcIntegration.inverse();
            
            Sophus::SE3f TwcNew =  pKF->GetPoseInverse();  // new corrected pose 
            
            // let's compute the correction transformation 
            //cv::Mat Twnwo= TwcNew * TwcIntegration.inv(); // from world old to world new             
            Sophus::SE3f Twnwo= TwcNew * TcwIntegration; // from world old to world new 

            // check if the transformation is "big" enough otherwise do not re-transform the cloud 
            //double norm = cv::norm(Twnwo - identity);
            double norm = (Twnwo.matrix3x4() - Sophus::SE3f().matrix3x4()).norm();
            std::cout << "kfid: " << kfid << ", norm: " << norm << std::endl; 
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
            //*(this->pPointCloud_) += kfCloudWorldNew; // push the corrected cloud             
            this->ReinsertCloud(pkfCloudWorldNew);
        }

        // set the counters of all the inserted points at the minimum threshold 
        typename OctreeType::LeafNodeIterator it, itEnd;
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10    
        for (it = octree_.leaf_begin(), itEnd = octree_.leaf_end(); it != itEnd; it++)
#else
        for (it = octree_.leaf_depth_begin(), itEnd = octree_.leaf_depth_end(); it != itEnd; ++it)
#endif         
        {
            //if(it.getCurrentOctreeDepth() == octree_.getTreeDepth())
            {
                typename OctreeType::LeafContainer& leaf = it.getLeafContainer();

                unsigned int& counter = leaf.getCounter(); 
                counter = this->pPointCloudMapParameters_->nPointCounterThreshold;
                
                PointT& mapPoint = leaf.getCentroid();
           
                this->pPointCloud_->push_back(mapPoint);                
            }
        }           
        
        //this->UpdateMap(); // filter and update timestamp 
        
        /// < update timestamp !
        this->UpdateMapTimestamp();
         
    }
    
}

template<typename PointT>
void PointCloudMapOctreePointCloud<PointT>::SetIntProperty(unsigned int property, int val)
{
    switch (property)
    {
    case PointCloudMapOctreePointCloud<PointT>::kPointCounterThreshold:
        this->pPointCloudMapParameters_->nPointCounterThreshold = val;
        break;
    default:
        ; //nop
    }
}

template<typename PointT>
bool PointCloudMapOctreePointCloud<PointT>::LoadMap(const std::string& filename)
{
    std::cout << "PointCloudMapOctreePointCloud<PointT>::LoadMap() - loading..." << std::endl;
    
    if( PointCloudMap<PointT>::LoadMap(filename) )  
    {
        std::unique_lock<std::recursive_timed_mutex> lck(this->pointCloudMutex_);

        this->InsertCloud(this->pPointCloud_);

        // set the counters of all the inserted points at the minimum threshold 
        typename OctreeType::LeafNodeIterator it, itEnd;
#if PCL_VERSION <= PCL_VERSION_CALC(1, 10, 0)  // pcl 1.10    
        for (it = octree_.leaf_begin(), itEnd = octree_.leaf_end(); it != itEnd; it++)
#else
        for (it = octree_.leaf_depth_begin(), itEnd = octree_.leaf_depth_end(); it != itEnd; ++it)
#endif          
        {
            //if(it.getCurrentOctreeDepth() == octree_.getTreeDepth())
            {
                typename OctreeType::LeafContainer& leaf = it.getLeafContainer();

                unsigned int& counter = leaf.getCounter(); 
                counter = this->pPointCloudMapParameters_->nPointCounterThreshold;
            }
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
