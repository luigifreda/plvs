/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "octomap_world/octomap_world.h"

#include <octomap_msgs/GetOctomap.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <volumetric_msgs/LoadMap.h>
#include <volumetric_msgs/SaveMap.h>
#include <volumetric_msgs/SetBoxOccupancy.h>
#include <volumetric_msgs/SetDisplayBounds.h>

#include <pcl_conversions/pcl_conversions.h>

namespace volumetric_mapping {

// An inherited class from OctomapWorld, which also handles the connection to
// ROS via publishers, subscribers, service calls, etc.
class OctomapManager : public OctomapWorld {
 public:
  typedef std::shared_ptr<OctomapManager> Ptr;

  // By default, loads octomap parameters from the ROS parameter server.
  OctomapManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void publishAll();
  void publishAllEvent(const ros::TimerEvent& e);

  // Data insertion callbacks with TF frame resolution through the listener.
  void insertDisparityImageWithTf(
      const stereo_msgs::DisparityImageConstPtr& disparity);
  void insertPointcloudWithTf(
      const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

  // Input Octomap callback.
  void octomapCallback(const octomap_msgs::Octomap& msg);

  // Camera info callbacks.
  void leftCameraInfoCallback(const sensor_msgs::CameraInfoPtr& left_info);
  void rightCameraInfoCallback(const sensor_msgs::CameraInfoPtr& right_info);

  // Service callbacks.
  bool resetMapCallback(std_srvs::Empty::Request& request,
                        std_srvs::Empty::Response& response);
  bool publishAllCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool getOctomapCallback(octomap_msgs::GetOctomap::Request& request,
                          octomap_msgs::GetOctomap::Response& response);

  bool loadOctomapCallback(volumetric_msgs::LoadMap::Request& request,
                           volumetric_msgs::LoadMap::Response& response);
  bool saveOctomapCallback(volumetric_msgs::SaveMap::Request& request,
                           volumetric_msgs::SaveMap::Response& response);
  bool savePointCloudCallback(volumetric_msgs::SaveMap::Request& request,
                              volumetric_msgs::SaveMap::Response& response);

  bool setBoxOccupancyCallback(
      volumetric_msgs::SetBoxOccupancy::Request& request,
      volumetric_msgs::SetBoxOccupancy::Response& response);
  bool setDisplayBoundsCallback(
      volumetric_msgs::SetDisplayBounds::Request& request,
      volumetric_msgs::SetDisplayBounds::Response& response);

  void transformCallback(const geometry_msgs::TransformStamped& transform_msg);

 private:
  // Sets up subscriptions based on ROS node parameters.
  void setParametersFromROS();
  void subscribe();
  void advertiseServices();
  void advertisePublishers();

  bool setQFromParams(std::vector<double>* Q_vec);
  void calculateQ();
  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const ros::Time& timestamp,
                       Transformation* transform);
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp, Transformation* transform);
  bool lookupTransformQueue(const std::string& from_frame,
                            const std::string& to_frame,
                            const ros::Time& timestamp,
                            Transformation* transform);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener tf_listener_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;
  std::string robot_frame_;
  // Whether to use TF transform resolution (true) or fixed transforms from
  // parameters and transform topics (false).
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;
  // B is the body frame of the robot, C is the camera/sensor frame creating
  // the pointclouds, and D is the 'dynamic' frame; i.e., incoming messages
  // are assumed to be T_G_D.
  Transformation T_B_C_;
  Transformation T_B_D_;

  bool latch_topics_;
  // Subscriptions for input sensor data.
  ros::Subscriber disparity_sub_;
  ros::Subscriber left_info_sub_;
  ros::Subscriber right_info_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber octomap_sub_;

  // Only used if use_tf_transforms_ set to false.
  ros::Subscriber transform_sub_;

  // Publish full state of octomap.
  ros::Publisher binary_map_pub_;
  ros::Publisher full_map_pub_;

  // Publish voxel centroids as pcl.
  ros::Publisher nearest_obstacle_pub_;
  ros::Publisher pcl_pub_;

  // Publish markers for visualization.
  ros::Publisher occupied_nodes_pub_;
  ros::Publisher free_nodes_pub_;

  // Services!
  ros::ServiceServer reset_map_service_;
  ros::ServiceServer publish_all_service_;
  ros::ServiceServer get_map_service_;
  ros::ServiceServer save_octree_service_;
  ros::ServiceServer load_octree_service_;
  ros::ServiceServer save_point_cloud_service_;
  ros::ServiceServer set_box_occupancy_service_;
  ros::ServiceServer set_display_bounds_service_;

  // Keep state of the cameras.
  sensor_msgs::CameraInfoPtr left_info_;
  sensor_msgs::CameraInfoPtr right_info_;

  // Only calculate Q matrix for disparity once.
  bool Q_initialized_;
  Eigen::Matrix4d Q_;
  Eigen::Vector2d full_image_size_;
  double map_publish_frequency_;
  ros::Timer map_publish_timer_;

  // Transform queue, used only when use_tf_transforms is false.
  std::deque<geometry_msgs::TransformStamped> transform_queue_;
};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
