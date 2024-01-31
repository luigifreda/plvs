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

#ifndef OCTOMAP_MANAGER_H_
#define OCTOMAP_MANAGER_H_

#define COMPILE_WITHOUT_ROS

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "octomap_world/octomap_world.h"

//#include <pcl_conversions/pcl_conversions.h>


namespace PLVS2
{

// An inherited class from OctomapWorld, which also handles the connection to
// ROS via publishers, subscribers, service calls, etc.

class OctomapManager : public volumetric_mapping::OctomapWorld
{
public:
    typedef std::shared_ptr<OctomapManager> Ptr;

    // By default, loads octomap default parameters 
    OctomapManager();

    //void publishAll();
    //void publishAllEvent(const ros::TimerEvent& e);

    // Data insertion callbacks with TF frame resolution through the listener.
    //void insertDisparityImageWithTf(const stereo_msgs::DisparityImageConstPtr& disparity);
    //void insertPointcloudWithTf(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

    // Input Octomap callback.
    //void octomapCallback(const octomap_msgs::Octomap& msg);

    // Camera info callbacks.
    //void leftCameraInfoCallback(const sensor_msgs::CameraInfoPtr& left_info);
    //void rightCameraInfoCallback(const sensor_msgs::CameraInfoPtr& right_info);

    // Service callbacks.
    //bool resetMapCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    //bool publishAllCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    //bool getOctomapCallback(octomap_msgs::GetOctomap::Request& request, octomap_msgs::GetOctomap::Response& response);

    bool loadOctomap(const std::string& file_path);
    
    bool saveOctomap(const std::string& file_path);
    
    bool savePointCloud(const std::string& file_path);

//    bool setBoxOccupancyCallback(
//                                 volumetric_msgs::SetBoxOccupancy::Request& request,
//                                 volumetric_msgs::SetBoxOccupancy::Response& response);
//    bool setDisplayBoundsCallback(
//                                  volumetric_msgs::SetDisplayBounds::Request& request,
//                                  volumetric_msgs::SetDisplayBounds::Response& response);
//
//    void transformCallback(const geometry_msgs::TransformStamped& transform_msg);
    
    void insertPointcloud(const volumetric_mapping::Transformation& T_G_sensor, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_sensor); 

private:
    // Sets up subscriptions based on ROS node parameters.
//    void setParametersFromROS();
//    void subscribe();
//    void advertiseServices();
//    void advertisePublishers();
//
//      bool setQFromParams(std::vector<double>* Q_vec);
//    void calculateQ();
//    bool lookupTransform(const std::string& from_frame,
//                         const std::string& to_frame, const ros::Time& timestamp,
//                         Transformation* transform);
//    bool lookupTransformTf(const std::string& from_frame,
//                           const std::string& to_frame,
//                           const ros::Time& timestamp, Transformation* transform);
//    bool lookupTransformQueue(const std::string& from_frame,
//                              const std::string& to_frame,
//                              const ros::Time& timestamp,
//                              Transformation* transform);


//    // Global/map coordinate frame. Will always look up TF transforms to this
//    // frame.
//    std::string world_frame_;
//    std::string robot_frame_;
//    // Whether to use TF transform resolution (true) or fixed transforms from
//    // parameters and transform topics (false).
//    // bool use_tf_transforms_;
//    int64_t timestamp_tolerance_ns_;
//    // B is the body frame of the robot, C is the camera/sensor frame creating
//    // the pointclouds, and D is the 'dynamic' frame; i.e., incoming messages
//    // are assumed to be T_G_D.
//    volumetric_mapping::Transformation T_B_C_;
//    volumetric_mapping::Transformation T_B_D_;
//
//    // Only calculate Q matrix for disparity once.
//    bool Q_initialized_;
//    Eigen::Matrix4d Q_;
//    Eigen::Vector2d full_image_size_;
//    double map_publish_frequency_;
    
};

} // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
