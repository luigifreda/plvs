/*
 * This file is part of PLVS.
 * This file is a modified version present in RGBDSLAM2 (https://github.com/felixendres/rgbdslam_v2)
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

#ifndef COLOR_OCTOMAP_SERVER_
#define COLOR_OCTOMAP_SERVER_

#include "PointDefinitions.h"  

//#include "parameter_server.h"
//#include <octomap/ColorVoxelMap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>

#include <pcl/common/transforms.h>

#include <memory>
#include <boost/shared_ptr.hpp>

 

namespace PLVS2
{

struct ColorOctomapParameters
{

    ColorOctomapParameters()
    : resolution(0.05),
    probability_hit(0.7),
    probability_miss(0.4),
    threshold_min(0.12),
    threshold_max(0.97),
    threshold_occupancy(0.55),
    filter_speckles(true),
    max_free_space(0.0),
    min_height_free_space(0.0),
    sensor_max_range(10.0),
    visualize_min_z(-std::numeric_limits<double>::max()),
    visualize_max_z(std::numeric_limits<double>::max()),
    treat_unknown_as_occupied(true),
    change_detection_enabled(false),
    display_level(16)
    {
        // Set reasonable defaults here...
    }

    // Resolution for the Octree. It is not possible to change this without
    // creating a new Octree.
    double resolution;
    // Hit probabilities for pointcloud data.
    double probability_hit;
    double probability_miss;
    // Clamping thresholds for pruning: above and below these thresholds, all
    // values are treated the same.
    double threshold_min;
    double threshold_max;
    // Threshold considered for a cell to be occupied.
    double threshold_occupancy;

    // Filter neighbor-less nodes as 'speckles'.
    bool filter_speckles;

    // Maximum range to allow a free space update.
    double max_free_space;

    // Minimum height below sensor to allow a free space update.
    double min_height_free_space;

    // Maximum range to allow a sensor measurement. Negative values to not
    // filter.
    double sensor_max_range;

    // Minimum and maximum z to visualize. Only used for marker, not full
    // octomap, visualization.
    double visualize_min_z;
    double visualize_max_z;

    // Collision checking.
    bool treat_unknown_as_occupied;

    // Whether to track changes -- must be set to true to use getChangedPoints().
    bool change_detection_enabled;
    
    // Display level 
    int display_level; 
};


template<typename PointT>
class ColorOctomapServer
{
public:     
    
    typedef typename pcl::PointCloud<PointT> PointcloudType;

public:
    ColorOctomapServer(const ColorOctomapParameters& params = ColorOctomapParameters());
    virtual ~ColorOctomapServer();
    void Reset();
    bool Save(const char* filename) const;
    
    ///Raycas cloud into the octomap
    /// @param cloud pointcloud in world frame 
    virtual void InsertCloudCallback(const typename PointcloudType::ConstPtr cloud_world, const octomap::point3d& origin, double max_range = -1.0);

    ///Raycast cloud into the octomap
    /// @param cloud pointcloud in map frame
    /// @param origin sensor location in map frame
    virtual void InsertCloudCallbackCommon(boost::shared_ptr<octomap::Pointcloud> octomapCloud,
                                           typename PointcloudType::ConstPtr color_cloud_world,
                                           const octomap::point3d& origin, double max_range = -1.0);

    ///Filter cloud by occupancy of voxels, e.g. remove points in free space
    void OccupancyFilter(typename PointcloudType::ConstPtr input,
                         typename PointcloudType::Ptr output,
                         double occupancy_threshold);

    virtual void Render();
    
    void GetOccupiedPointCloudLevel(typename PointcloudType::Ptr output_cloud) const;
    void GetOccupiedPointCloud(typename PointcloudType::Ptr output_cloud) const;
        
protected:
    octomap::ColorOcTree octoMap_;
    //octomap::OctomapROS<octomap::ColorOcTree> m_octoMap;
    //mutable QFuture<void> rendering; //Mutable is a hack, otherwise waitforfinished cannot be called in const function
    
    ColorOctomapParameters params_; 
};


/// < list here the types you want to use 
template class ColorOctomapServer<pcl::PointXYZRGBA>;
template class ColorOctomapServer<pcl::PointXYZRGBNormal>;
template class ColorOctomapServer<pcl::PointSurfelSegment>;

} //namespace PLVS2


#endif

