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

#ifndef VOXEL_GRID_CUSTOM_H
#define VOXEL_GRID_CUSTOM_H 

#include "PointDefinitions.h"

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>


namespace pcl
{

  template <typename PointT>
  class VoxelGridCustom: public VoxelGrid<PointT>
  {

    public:
    using Filter<PointT>::filter_name_;
    
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;    
    
    public:
      /** \brief Empty constructor. */
      VoxelGridCustom()
      {
        filter_name_ = "VoxelGridCustom";
      }

      /** \brief Destructor. */
      virtual ~VoxelGridCustom ()
      {
      }
      
    protected:      
      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param[out] output the resultant point cloud message
        */
      void applyFilter (PointCloud &output)
      {
          VoxelGrid<PointT>::applyFilter(output);
      }

  };
  
  
  template <>
  class VoxelGridCustom<pcl::PointSurfelSegment>: public VoxelGrid<pcl::PointSurfelSegment>
  {
    
    public:   
    typedef pcl::PointSurfelSegment PointT;
      
    using Filter<PointT>::filter_name_;
    
    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;   
    
    public:
      /** \brief Empty constructor. */
      VoxelGridCustom()
      {
        filter_name_ = "VoxelGridCustom";
      }

      /** \brief Destructor. */
      virtual ~VoxelGridCustom ()
      {
      }
            
    protected:      
      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param[out] output the resultant point cloud message
        */   
      void applyFilter (PointCloud &output);      

  };  
  
}


#include "VoxelGridCustom.hpp"


#endif 