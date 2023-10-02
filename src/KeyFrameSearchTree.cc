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


#include "KeyFrameSearchTree.h"
#include "KeyFrame.h"


namespace PLVS2
{

template<typename PointT>
KeyFrameSearchTree<PointT>::KeyFrameSearchTree(float resolution, float searchRange):
mfResolution(resolution), mfSearchRange(searchRange)
{
    mpCloud.reset( new PointCloud());
    mpSearchTree.reset( new SearchTree(resolution));
    
    mpSearchTree->setInputCloud(mpCloud);
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::AddKeyFrame(KeyFramePtr& pKF)
{    
    //cv::Mat Ow = pKF->GetCameraCenter();
    Eigen::Vector3f Ow = pKF->GetFovCenter();
    
    PointT point;
    point.x = Ow(0);
    point.y = Ow(1);    
    point.z = Ow(2); 
    point.label = pKF->mnId;  //  label is used to store the KeyFrame Id
    
    this->AddPoint(point);
    
    std::unique_lock<std::recursive_timed_mutex> lck(mMutex);     
    mvKFs.push_back(pKF);
    
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::GetCloseKeyFrames(KeyFramePtr& pKF, std::set<KeyFramePtr>& setActiveKFs, std::set<uint32_t> &setIds)
{
    //cv::Mat Ow = pKF->GetCameraCenter();
    Eigen::Vector3f Ow = pKF->GetFovCenter();
    
    PointT point;
    point.x = Ow(0);
    point.y = Ow(1);    
    point.z = Ow(2); 
    point.label = pKF->mnId;  //  label is used to store the KeyFrame Id    
    
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    this->RadiusSearch(point, mfSearchRange, k_indices, k_sqr_distances);
    
    setActiveKFs.clear();
    setIds.clear();
    for(size_t ii=0, iiEnd=k_indices.size(); ii<iiEnd; ii++)
    {
        setActiveKFs.insert(mvKFs[ii]);
        setIds.insert(mpCloud->points[ii].label);
        //vActiveKFs[ii] = mvKFs[ii];
        //vIds[ii] = mpCloud->points[ii].label; 
    }
    
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::SetSearchRange(float range)
{
    mfSearchRange = range; 
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::AddPoint(const PointT& point)
{
    std::unique_lock<std::recursive_timed_mutex> lck(mMutex); 
    
    // this add the point simultaneously to octree and input point cloud.
    mpSearchTree->addPointToCloud(point, mpCloud);
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::AddPoint(const Eigen::Vector3f& Ow)
{
    PointT point;
    point.x = Ow(0);
    point.y = Ow(1);    
    point.z = Ow(2);    
    this->AddPoint(point);
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::RadiusSearch(const PointT& point, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
{
    std::unique_lock<std::recursive_timed_mutex> lck(mMutex);     
    mpSearchTree->radiusSearch(point, radius, k_indices, k_sqr_distances);
}

template<typename PointT>
void KeyFrameSearchTree<PointT>::RadiusSearch(const Eigen::Vector3f& Ow, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
{
    PointT point;
    point.x = Ow(0);
    point.y = Ow(1);    
    point.z = Ow(2);        
    this->RadiusSearch(point, radius, k_indices, k_sqr_distances);
}

} //namespace PLVS2



