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
/**
 * partly taken from the ros octomap server.
 * @author A. Hornung, K. M. Wurm University of Freiburg, Copyright (C) 2009.
 * License: GPL
 */
/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
 * All rights reserved.
 */


#include "ColorOctomapServer.h"
//#include "scoped_timer.h"
//#include <pcl_ros/transforms.h>
//#include <pcl_ros/impl/transforms.hpp>
#include <GL/gl.h>

namespace PLVS2
{

template<typename PointT>
ColorOctomapServer<PointT>::ColorOctomapServer(const ColorOctomapParameters& params) : octoMap_(params.resolution), params_(params)
{
    Reset();
}

template<typename PointT>
ColorOctomapServer<PointT>::~ColorOctomapServer()
{
}

///Clear octomap and reset values to paramters from parameter server
template<typename PointT>
void ColorOctomapServer<PointT>::Reset()
{
    this->octoMap_.clear();
    this->octoMap_.setClampingThresMin(params_.threshold_min);
    this->octoMap_.setClampingThresMax(params_.threshold_max);
    this->octoMap_.setResolution(params_.resolution);
    this->octoMap_.setOccupancyThres(params_.threshold_occupancy);
    this->octoMap_.setProbHit(params_.probability_hit);
    this->octoMap_.setProbMiss(params_.probability_miss);
    this->octoMap_.enableChangeDetection(params_.change_detection_enabled);
}

template<typename PointT>
bool ColorOctomapServer<PointT>::Save(const char* filename) const
{
    //ScopedTimer s(__FUNCTION__);
    std::ofstream outfile(filename, std::ios_base::out | std::ios_base::binary);
    if (outfile.is_open())
    {
        //        //m_octoMap.writeConst(outfile); 
        //        if (ParameterServer::instance()->get<bool>("concurrent_io"))
        //        {
        //            ROS_INFO("Waiting for rendering thread to finish");
        //            rendering.waitForFinished();
        //        }
        std::cout << "Writing octomap to " << filename << std::endl;
        octoMap_.write(outfile);
        outfile.close();
        std::cout << "color tree written :" << filename << std::endl;
        return true;
    }
    else
    {
        std::cout << "could not open file " << filename << std::endl;
        return false;
    }
}

//Same as the other insertCloudCallback, but relies on the sensor position information in the cloud

template<typename PointT>
void ColorOctomapServer<PointT>::InsertCloudCallback(const typename PointcloudType::ConstPtr cloud_world, const octomap::point3d& origin, double max_range)
{

    //    ScopedTimer s(__FUNCTION__);

    //Conversions
    //    Eigen::Quaternionf q = cloud->sensor_orientation_;
    //    Eigen::Vector4f t = cloud->sensor_origin_;
    //    tf::Transform trans(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(t[0], t[1], t[2]));
    //    octomap::point3d origin(t[0], t[1], t[2]);
    //    pointcloud_type::Ptr pcl_cloud(new pointcloud_type);
    //
    //    //Work
    //    pcl_ros::transformPointCloud(*cloud, *pcl_cloud, trans);

    //Conversions
    boost::shared_ptr<octomap::Pointcloud> octomapCloud(new octomap::Pointcloud());

    //Work 
    octomapCloud->reserve(cloud_world->size());
    for (typename PointcloudType::const_iterator it = cloud_world->begin(), itEnd = cloud_world->end(); it != itEnd; ++it)
    {
#if 0        
        if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
#endif            
        {
            octomapCloud->push_back(it->x, it->y, it->z);
        }
    }

    //    if (ParameterServer::instance()->get<bool>("concurrent_io"))
    //    {
    //        rendering.waitForFinished();
    //        rendering = QtConcurrent::run(this, &ColorOctomapServer::insertCloudCallbackCommon, octomapCloud, pcl_cloud, origin, max_range);
    //    }
    //    else
    {
        InsertCloudCallbackCommon(octomapCloud, cloud_world, origin, max_range);
    }
}

template<typename PointT>
void ColorOctomapServer<PointT>::InsertCloudCallbackCommon(boost::shared_ptr<octomap::Pointcloud> octomapCloud,
                                                   typename PointcloudType::ConstPtr color_cloud_world,
                                                   const octomap::point3d& origin, double max_range)
{
    //    if (m_octoMap.getResolution() != ParameterServer::instance()->get<double>("octomap_resolution"))
    //    {
    //        ROS_WARN("OctoMap resolution changed from %f to %f. Resetting Octomap",
    //                 m_octoMap.getResolution(), ParameterServer::instance()->get<double>("octomap_resolution"));
    //        this->reset();
    //    }
    //geometry_msgs::Point origin;
    //tf::pointTFToMsg(trans.getOrigin(), origin);

    std::cout << "ColorOctomapServer::insertCloudCallbackCommon() - inserting data" << std::endl;
    octoMap_.insertPointCloud(*octomapCloud, origin, max_range, true);
    // integrate color measurements
    //unsigned char* colors = new unsigned char[3];

    std::cout << "ColorOctomapServer::insertCloudCallbackCommon() - inserting color measurements" << std::endl;
    typename PointcloudType::const_iterator it, itEnd;
    for (it = color_cloud_world->begin(), itEnd = color_cloud_world->end(); it != itEnd; ++it)
    {
        // Check if the point is invalid
        //if (!std::isnan(it->x) && !std::isnan(it->y) && !std::isnan(it->z))
        {

//            const int rgb = *reinterpret_cast<const int*> (&(it->rgb));
//
//            colors[0] = ((rgb >> 16) & 0xff);
//            colors[1] = ((rgb >> 8) & 0xff);
//            colors[2] = (rgb & 0xff);

            
            octoMap_.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
        }
    }

    // updates inner node colors, too
    std::cout << "ColorOctomapServer::insertCloudCallbackCommon() - updating inner nodes" << std::endl;
    octoMap_.updateInnerOccupancy();
}

//Filter, e.g. points in free space
template<typename PointT>
void ColorOctomapServer<PointT>::OccupancyFilter(typename PointcloudType::ConstPtr input,
                                         typename PointcloudType::Ptr output,
                                         double occupancy_threshold)
{
    if (output->points.capacity() < input->size())
    { //cannot happen for input == output
        output->reserve(input->size());
    }

    Eigen::Quaternionf q = input->sensor_orientation_;
    Eigen::Vector4f t = input->sensor_origin_;

    size_t size = input->size();
    size_t outidx = 0;
    for (size_t inidx = 0; inidx < size; ++inidx)
    {
        const PointT& in_point = (*input)[inidx];
        Eigen::Vector3f in_vec = q * in_point.getVector3fMap() + t.head<3>();
        if (std::isnan(in_vec.z()))
            continue;

        const int radius = 1;
        int x_a = octoMap_.coordToKey(in_vec.x()) - radius;
        int x_b = octoMap_.coordToKey(in_vec.x()) + radius;
        int y_a = octoMap_.coordToKey(in_vec.y()) - radius;
        int y_b = octoMap_.coordToKey(in_vec.y()) + radius;
        int z_a = octoMap_.coordToKey(in_vec.z()) - radius;
        int z_b = octoMap_.coordToKey(in_vec.z()) + radius;
        double sum_of_occupancy = 0, sum_of_weights = 0;
        for (; x_a <= x_b; ++x_a)
        {
            for (; y_a <= y_b; ++y_a)
            {
                for (; z_a <= z_b; ++z_a)
                {
                    octomap::OcTreeNode* node = octoMap_.search(octomap::OcTreeKey(x_a, y_a, z_a));
                    if (node != NULL)
                    {
                        double dx = octoMap_.keyToCoord(x_a) - in_vec.x();
                        double dy = octoMap_.keyToCoord(y_a) - in_vec.y();
                        double dz = octoMap_.keyToCoord(z_a) - in_vec.z();
                        double weight = dx * dx + dy * dy + dz*dz;
                        double weighted_occ = node->getOccupancy() / weight;
                        sum_of_weights += weight;
                        sum_of_occupancy += weighted_occ;
                    }
                }
            }
        }
 

        if (sum_of_occupancy < occupancy_threshold * sum_of_weights) //Filters points in non-existent nodes (outside of map?) 
            //if(node != NULL && node->getOccupancy() >= occupancy_threshold) 
        { //Valid point
            PointT& out_point = (*output)[outidx];
            out_point = in_point;
            ++outidx;
        }
    }
    output->resize(outidx); //downsize
}

template<typename PointT>
void ColorOctomapServer<PointT>::Render()
{
    octomap::ColorOcTree::tree_iterator it = octoMap_.begin_tree();
    octomap::ColorOcTree::tree_iterator end = octoMap_.end_tree();
    int counter = 0;
    double occ_thresh = params_.threshold_occupancy;
    int level = params_.display_level;
    if (occ_thresh > 0)
    {
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        //glDisable(GL_CULL_FACE);
        glBegin(GL_TRIANGLES);
        double stretch_factor = 128 / (1 - occ_thresh); //occupancy range in which the displayed cubes can be
        for (; it != end; ++counter, ++it)
        {
            if (level != it.getDepth())
            {
                continue;
            }
            double occ = it->getOccupancy();
            if (occ < occ_thresh)
            {
                continue;
            }
            glColor4ub(it->getColor().r, it->getColor().g, it->getColor().b, 128 /*basic visibility*/ + (occ - occ_thresh) * stretch_factor);
            float halfsize = it.getSize() / 2.0;
            float x = it.getX();
            float y = it.getY();
            float z = it.getZ();
            //Front
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);

            //Back
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);

            //Left
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);

            //Right
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            //?
            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);

            glVertex3f(x - halfsize, y - halfsize, z - halfsize);
            glVertex3f(x + halfsize, y - halfsize, z + halfsize);
            glVertex3f(x - halfsize, y - halfsize, z + halfsize);

            //?
            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x - halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);

            glVertex3f(x - halfsize, y + halfsize, z - halfsize);
            glVertex3f(x + halfsize, y + halfsize, z + halfsize);
            glVertex3f(x + halfsize, y + halfsize, z - halfsize);
        }
        glEnd();
    }
}

template<typename PointT>
void ColorOctomapServer<PointT>::GetOccupiedPointCloudLevel(typename PointcloudType::Ptr output_cloud) const
{
    if(output_cloud)
    {
        output_cloud->clear();
    }
    else
    {
        output_cloud.reset(new PointcloudType());
    }
        
    octomap::ColorOcTree::tree_iterator it  = octoMap_.begin_tree();
    octomap::ColorOcTree::tree_iterator end = octoMap_.end_tree();
    int counter = 0;
    double occ_thresh = params_.threshold_occupancy;
    int level = params_.display_level;
    if (occ_thresh > 0)
    {
        double stretch_factor = 128 / (1 - occ_thresh); //occupancy range in which the displayed cubes can be
        for (; it != end; ++counter, ++it)
        {
            if (level != it.getDepth())
            {
                continue;
            }
            double occ = it->getOccupancy();
            if (occ < occ_thresh)
            {
                continue;
            }
 
            PointT point;
            point.x = it.getX();
            point.y = it.getY();
            point.z = it.getZ();
            point.r = it->getColor().r;
            point.g = it->getColor().g;
            point.b = it->getColor().b;
            point.a = 128 /*basic visibility*/ + (occ - occ_thresh) * stretch_factor;

            output_cloud->push_back(point);
        }
    }
}

template<typename PointT>
void ColorOctomapServer<PointT>::GetOccupiedPointCloud(typename PointcloudType::Ptr output_cloud) const
{
    if(output_cloud)
    {
        output_cloud->clear();
    }
    else
    {
        output_cloud.reset(new PointcloudType());
    }

    double occ_thresh = params_.threshold_occupancy;

    if (occ_thresh <= 0) return; /// < EXIT POINT 
    
    double stretch_factor = 128 / (1 - occ_thresh); //occupancy range in which the displayed cubes can be
    
    unsigned int max_tree_depth = octoMap_.getTreeDepth();
    double resolution = octoMap_.getResolution();
    for (octomap::ColorOcTree::leaf_iterator it = octoMap_.begin_leafs(); it != octoMap_.end_leafs(); ++it)
    {
        if (octoMap_.isNodeOccupied(*it))
        {
            // If leaf is max depth add coordinates.
            if (max_tree_depth == it.getDepth())
            {
                PointT point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                point.r = it->getColor().r;
                point.g = it->getColor().g;
                point.b = it->getColor().b;
                point.a = 128 /*basic visibility*/ + (it->getOccupancy() - occ_thresh) * stretch_factor;
                output_cloud->push_back(point);
            }
                // If leaf is not max depth it represents an occupied voxel with edge
                // length of 2^(max_tree_depth - leaf_depth) * resolution.
                // We use multiple points to visualize this filled volume.
            else
            {
                const unsigned int box_edge_length = pow(2, max_tree_depth - it.getDepth() - 1);
                const double bbx_offset = box_edge_length * resolution - resolution / 2;
                Eigen::Vector3d bbx_offset_vec(bbx_offset, bbx_offset, bbx_offset);
                Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
                Eigen::Vector3d bbx_min = center - bbx_offset_vec;
                Eigen::Vector3d bbx_max = center + bbx_offset_vec;
                // Add small offset to avoid overshooting bbx_max.
                bbx_max += Eigen::Vector3d(0.001, 0.001, 0.001);
                for (double x_position = bbx_min.x(); x_position <= bbx_max.x(); x_position += resolution)
                {
                    for (double y_position = bbx_min.y(); y_position <= bbx_max.y(); y_position += resolution)
                    {
                        for (double z_position = bbx_min.z(); z_position <= bbx_max.z(); z_position += resolution)
                        {
                            PointT point;
                            point.x = x_position;
                            point.y = y_position;
                            point.z = z_position;
                            point.r = it->getColor().r;
                            point.g = it->getColor().g;
                            point.b = it->getColor().b;
                            point.a = 128 /*basic visibility*/ + (it->getOccupancy() - occ_thresh) * stretch_factor;
                            
                            output_cloud->push_back(point);
                        }
                    }
                }
            }
        }
    }
    
    
} 


} //namespace PLVS2