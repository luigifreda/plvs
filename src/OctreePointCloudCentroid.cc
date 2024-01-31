/*
 * This file is part of PLVS.
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

#include "OctreePointCloudCentroid.h"

namespace pcl
{
namespace octree
{


template<typename PointT>
const unsigned int OctreePointCloudVoxelCentroidContainerMV<PointT>::kMaxPointCounterValForMovingAverage = 10;

template<typename PointT>
const unsigned int OctreePointCloudVoxelCentroidContainerMV<PointT>::kMaxPointCounterValForNormalMovingAverage = 10;

template<typename PointT>
const int OctreePointCloudVoxelCentroidContainerMV<PointT>::kMaxLabelConfidence = 10; // the higher this value, the less a label is dynamic and available to be changed/updated/corrected


template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointIdx(const int point_idx_arg, const std::uint64_t& time_stamp)
{
    OctreeKey key;

    assert(point_idx_arg < static_cast<int> (this->input_->points.size()));

    const PointT& point = this->input_->points[point_idx_arg];

    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(point);

    // generate key
    this->genOctreeKeyforPoint(point, key);

    LeafNode* leaf_node;
    BranchNode* parent_branch_of_leaf_node;
    unsigned int depth_mask = this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);

    if (this->dynamic_depth_enabled_ && depth_mask)
    {
        // get amount of objects in leaf container
        size_t leaf_obj_count = (*leaf_node)->getSize();

        while (leaf_obj_count >= this->max_objs_per_leaf_ && depth_mask)
        {
            // index to branch child
            unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask * 2);

            this->expandLeafNode(leaf_node,
                                 parent_branch_of_leaf_node,
                                 child_idx,
                                 depth_mask);

            depth_mask = this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);
            leaf_obj_count = (*leaf_node)->getSize();
        }

    }

    (*leaf_node)->addPointIndex(point_idx_arg); // useless ? 

    (*leaf_node)->addPoint(point, time_stamp);
}

template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::reinserPointIdx(const int point_idx_arg, const std::uint64_t& time_stamp)
{
    OctreeKey key;

    assert(point_idx_arg < static_cast<int> (this->input_->points.size()));

    const PointT& point = this->input_->points[point_idx_arg];

    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(point);

    // generate key
    this->genOctreeKeyforPoint(point, key);

    LeafNode* leaf_node;
    BranchNode* parent_branch_of_leaf_node;
    unsigned int depth_mask = this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);

    if (this->dynamic_depth_enabled_ && depth_mask)
    {
        // get amount of objects in leaf container
        size_t leaf_obj_count = (*leaf_node)->getSize();

        while (leaf_obj_count >= this->max_objs_per_leaf_ && depth_mask)
        {
            // index to branch child
            unsigned char child_idx = key.getChildIdxWithDepthMask(depth_mask * 2);

            this->expandLeafNode(leaf_node,
                                 parent_branch_of_leaf_node,
                                 child_idx,
                                 depth_mask);

            depth_mask = this->createLeafRecursive(key, this->depth_mask_, this->root_node_, leaf_node, parent_branch_of_leaf_node);
            leaf_obj_count = (*leaf_node)->getSize();
        }

    }

    (*leaf_node)->addPointIndex(point_idx_arg); // useless ? 

    (*leaf_node)->reinsertPoint(point, time_stamp);
}

///

template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::addPointsFromInputCloud()
{
    size_t i, iEnd;

    std::uint64_t time_stamp = this->input_->header.stamp;

    if (this->indices_)
    {
        for (std::vector<int>::const_iterator current = this->indices_->begin(); current != this->indices_->end(); ++current)
        {
            assert((*current >= 0) && (*current < static_cast<int> (this->input_->points.size())));

            if (isFinite(this->input_->points[*current]))
            {
                // add points to octree
                this->addPointIdx(*current, time_stamp);
            }
        }
    }
    else
    {
        for (i = 0, iEnd=this->input_->points.size(); i < iEnd; i++)
        {
            if (isFinite(this->input_->points[i]))
            {
                // add points to octree
                this->addPointIdx(static_cast<unsigned int> (i), time_stamp);
            }
        }
    }
}

template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::reinsertPointsFromInputCloud()
{
    size_t i, iEnd;

    std::uint64_t time_stamp = this->input_->header.stamp;

    if (this->indices_)
    {
        for (std::vector<int>::const_iterator current = this->indices_->begin(); current != this->indices_->end(); ++current)
        {
            assert((*current >= 0) && (*current < static_cast<int> (this->input_->points.size())));

            if (isFinite(this->input_->points[*current]))
            {
                // add points to octree
                this->reinserPointIdx(*current, time_stamp);
            }
        }
    }
    else
    {
        for (i = 0, iEnd=this->input_->points.size(); i < iEnd; i++)
        {
            if (isFinite(this->input_->points[i]))
            {
                // add points to octree
                this->reinserPointIdx(static_cast<unsigned int> (i), time_stamp);
            }
        }
    }
}

///

template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
int OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::boxSearch(const Eigen::Vector3f &min_pt,
                                                                                           const Eigen::Vector3f &max_pt,
                                                                                           pcl::PointCloud<PointT>& cloud) const
{

    OctreeKey key;
    key.x = key.y = key.z = 0;

    cloud.clear();

    this->boxSearchRecursive(min_pt, max_pt, this->root_node_, key, 1, cloud);

    return (static_cast<int> (cloud.size()));

}

    

template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::boxSearchRecursive(const Eigen::Vector3f &min_pt,
                                                                                                     const Eigen::Vector3f &max_pt,
                                                                                                     const BranchNode* node,
                                                                                                     const OctreeKey& key,
                                                                                                     unsigned int tree_depth,
                                                                                                     pcl::PointCloud<PointT>& cloud) const
{
    // child iterator
    unsigned char child_idx;

    // iterate over all children
    for (child_idx = 0; child_idx < 8; child_idx++)
    {

        const OctreeNode* child_node;
        child_node = this->getBranchChildPtr(*node, child_idx);

        if (!child_node)
            continue;

        OctreeKey new_key;
        // generate new key for current branch voxel
        new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
        new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
        new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));

        // voxel corners
        Eigen::Vector3f lower_voxel_corner;
        Eigen::Vector3f upper_voxel_corner;
        // get voxel coordinates
        this->genVoxelBoundsFromOctreeKey(new_key, tree_depth, lower_voxel_corner, upper_voxel_corner);

        // test if search region overlap with voxel space
        if ((min_pt(0) <= upper_voxel_corner(0)) && (lower_voxel_corner(0) <= max_pt(0)) &&
            (min_pt(1) <= upper_voxel_corner(1)) && (lower_voxel_corner(1) <= max_pt(1)) &&
            (min_pt(2) <= upper_voxel_corner(2)) && (lower_voxel_corner(2) <= max_pt(2)))
        {

            if (tree_depth < this->octree_depth_)
            {
                // we have not reached maximum tree depth
                boxSearchRecursive(min_pt, max_pt, static_cast<const BranchNode*> (child_node), new_key, tree_depth + 1, cloud);
            }
            else
            {
                // we reached leaf node level
                //size_t i;
                //std::vector<int> decoded_point_vector;
                bool bInBox = true;

                const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);

                // decode leaf node into decoded_point_vector
                //(**child_leaf).getPointIndices (decoded_point_vector);
                const PointT& candidate_point = (**child_leaf).getCentroid();

                // Linearly iterate over all decoded (unsorted) points
                //for (i = 0; i < decoded_point_vector.size (); i++)
                {
                    //const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

                    // check if point falls within search box
                    //          bInBox = ( (candidate_point.x >= min_pt (0)) && (candidate_point.x <= max_pt (0)) &&
                    //                     (candidate_point.y >= min_pt (1)) && (candidate_point.y <= max_pt (1)) &&
                    //                     (candidate_point.z >= min_pt (2)) && (candidate_point.z <= max_pt (2)) );

                    if (bInBox)
                    {
                        // add to result vector
                        //k_indices.push_back (decoded_point_vector[i]);
                        cloud.push_back(candidate_point);
                    }
                }
            }
        }
    }
}

//
//template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT>
//void OctreePointCloudCentroid<PointT, LeafContainerT, BranchContainerT, OctreeT>::boxSearchRecursive (const Eigen::Vector3f &min_pt,
//                                                                                 const Eigen::Vector3f &max_pt,
//                                                                                 const BranchNode* node,
//                                                                                 const OctreeKey& key,
//                                                                                 unsigned int tree_depth,
//                                                                                 pcl::PointCloud<PointT>& cloud,
//                                                                                 unsigned int search_level) const
//{
//  // child iterator
//  unsigned char child_idx;
//
//  // iterate over all children
//  for (child_idx = 0; child_idx < 8; child_idx++)
//  {
//
//    const OctreeNode* child_node;
//    child_node = this->getBranchChildPtr (*node, child_idx);
//
//    if (!child_node)
//      continue;
//
//    OctreeKey new_key;
//    // generate new key for current branch voxel
//    new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
//    new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
//    new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));
//
//    // voxel corners
//    Eigen::Vector3f lower_voxel_corner;
//    Eigen::Vector3f upper_voxel_corner;
//    // get voxel coordinates
//    this->genVoxelBoundsFromOctreeKey (new_key, tree_depth, lower_voxel_corner, upper_voxel_corner);
//
//    // test if search region overlap with voxel space
//    if (  (min_pt (0) <= upper_voxel_corner(0)) && (lower_voxel_corner (0) <= max_pt (0)) && 
//          (min_pt (1) <= upper_voxel_corner(1)) && (lower_voxel_corner (1) <= max_pt (1)) && 
//          (min_pt (2) <= upper_voxel_corner(2)) && (lower_voxel_corner (2) <= max_pt (2))   )
//    {
//
//      int tree_depth_search_level = std::min(search_level, this->octree_depth_);
//      if (tree_depth < tree_depth_search_level)
//      {
//        // we have not reached maximum tree depth
//        boxSearchRecursive (min_pt, max_pt, static_cast<const BranchNode*> (child_node), new_key, tree_depth + 1, cloud);
//      }
//      else
//      {
//        // we reached leaf node level
//        //size_t i;
//        //std::vector<int> decoded_point_vector;
//        PointT candidate_point; 
//        bool bInBox = true;
//
//        const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);
//
//        // decode leaf node into decoded_point_vector
//        //(**child_leaf).getPointIndices (decoded_point_vector);
//        (**child_leaf).getCentroid (candidate_point);
//
//        // Linearly iterate over all decoded (unsorted) points
//        //for (i = 0; i < decoded_point_vector.size (); i++)
//        {
//          //const PointT& candidate_point = this->getPointByIndex (decoded_point_vector[i]);
//
//          // check if point falls within search box
////          bInBox = ( (candidate_point.x >= min_pt (0)) && (candidate_point.x <= max_pt (0)) &&
////                     (candidate_point.y >= min_pt (1)) && (candidate_point.y <= max_pt (1)) &&
////                     (candidate_point.z >= min_pt (2)) && (candidate_point.z <= max_pt (2)) );
//          
//          if (bInBox)
//          {
//            // add to result vector
//            //k_indices.push_back (decoded_point_vector[i]);
//            cloud.push_back(candidate_point);
//          }
//        }
//      }
//    }
//  }
//}


}//namespace octree
}//namespace pcl
