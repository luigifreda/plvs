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

#ifndef KEYFRAME_SEARCH_TREE_H
#define KEYFRAME_SEARCH_TREE_H

#include "PointDefinitions.h"
#include "PointCloudKeyFrame.h"

#include <pcl/octree/octree_search.h>

#include <mutex>


namespace PLVS2
{

class KeyFrame; 

///	\class KeyFrameSearchTree
///	\author Luigi Freda
///	\brief A class for "searching" close KeyFrame and identifying the active dense map 
///	\note
///	\date
///	\warning
template<typename PointT=pcl::PointXYZL>
class KeyFrameSearchTree
{
public:
    
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    
    typedef typename pcl::octree::OctreePointCloudSearch<PointT> SearchTree;
    typedef typename SearchTree::Ptr SearchTreePtr;    
            
public:
  
    KeyFrameSearchTree(float resolution = 0.05, float searchRange = 1.);

    void AddKeyFrame(KeyFramePtr& pKF);
    
    void GetCloseKeyFrames(KeyFramePtr& pKF, std::set<KeyFramePtr>& setActiveKFs, std::set<uint32_t> &setIds);
    
public: // setters 
    
    void SetSearchRange(float range); 
        
protected:    
    
    void AddPoint(const PointT& point);       
    void AddPoint(const Eigen::Vector3f& Ow);          
    
    void RadiusSearch(const PointT& point, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances);
    void RadiusSearch(const Eigen::Vector3f& Ow, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances);    
        
protected:
    
    PointCloudPtr mpCloud;    
    SearchTreePtr mpSearchTree;
    std::vector<KeyFramePtr> mvKFs;
    
    float mfResolution; 
    float mfSearchRange = 1.; // [meters] 
    
    std::recursive_timed_mutex mMutex;

};

/// < list here the types you want to use 
template class KeyFrameSearchTree<pcl::PointXYZL>;


} //namespace PLVS2

#endif /* KEYFRAME_SEARCH_TREE_H */

