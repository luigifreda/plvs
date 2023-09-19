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

#ifndef VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
#define VOLUMETRIC_MAP_BASE_WORLD_BASE_H_

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <kindr/minimal/quat-transformation.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#ifndef COMPILE_WITHOUT_ROS
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#endif

#include <pcl/conversions.h>
#include <Eigen/StdVector>

#include "volumetric_map_base/point_weighing.h"

namespace volumetric_mapping {

typedef kindr::minimal::QuatTransformation Transformation;

// Base class for all 3D volumetric representations of the environment.
// By default, implements a valid completely empty world.
class WorldBase {
 public:
     
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef std::shared_ptr<WorldBase> Ptr;

  enum CellStatus { kFree = 0, kOccupied = 1, kUnknown = 2 };

  WorldBase() {}
  virtual ~WorldBase() {}

  
#ifndef COMPILE_WITHOUT_ROS  
  // Data insertion functions.
  // Project the given disparity map to 3D and insert it into the map.
  // Q is a 4x4 perspective disparity-to-depth mapping matrix for the full-size
  // image. This takes care of adjusting Q for downsampled disparity maps as
  // well.
  // See: http://docs.opencv.org/modules/calib3d/doc/
  //      camera_calibration_and_3d_reconstruction.html#reprojectimageto3d
  void insertDisparityImage(
      const Transformation& sensor_to_world,
      const stereo_msgs::DisparityImageConstPtr& disparity,
      const Eigen::Matrix4d& Q_full, const Eigen::Vector2d& full_image_size);
  
#endif  
  void insertDisparityImage(const Transformation& sensor_to_world,
                            const cv::Mat& disparity,
                            const Eigen::Matrix4d& Q_full,
                            const Eigen::Vector2d& full_image_size);

  
  // Helper functions to compute the Q matrix for given camera parameters.
  // Assumes UNRECTIFIED camera matrices.
  // Downsampling is handled in insertDisparityImage.
  Eigen::Matrix4d getQForCameras(const Transformation& T_C1_C0,
                                 const Eigen::Matrix3d& left_cam_matrix,
                                 const Eigen::Matrix3d& right_cam_matrix,
                                 const Eigen::Vector2d& full_image_size) const;
  
#ifndef COMPILE_WITHOUT_ROS  
  Eigen::Matrix4d getQForROSCameras(
      const sensor_msgs::CameraInfo& left_camera,
      const sensor_msgs::CameraInfo& right_camera) const;
  // Calls insertPointcloudImpl() or insertPointcloudIntoMapWithWeightsImpl(),
  // depending if points are to be weighted.
  void insertPointcloud(
      const Transformation& T_G_sensor,
      const sensor_msgs::PointCloud2::ConstPtr& pointcloud_sensor);
#endif   
  void insertPointcloud(const Transformation& T_G_sensor,
                        const Eigen::Matrix3Xd& pointcloud_sensor);
  
  void insertPointcloud(
      const Transformation& T_G_sensor,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_sensor);

  // Manually affect the state of a bounding box. For the WorldBase class,
  // setting to occupied is a no-op.
  virtual void setFree(const Eigen::Vector3d& position,
                       const Eigen::Vector3d& bounding_box_size) {}
  virtual void setOccupied(const Eigen::Vector3d& position,
                           const Eigen::Vector3d& bounding_box_size) {}

  // Methods to query the current map state.
  virtual CellStatus getCellStatusBoundingBox(
      const Eigen::Vector3d& point,
      const Eigen::Vector3d& bounding_box_size) const {
    return CellStatus::kFree;
  }
  virtual CellStatus getCellStatusPoint(const Eigen::Vector3d& point) const {
    return CellStatus::kFree;
  }

  virtual CellStatus getLineStatus(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& end) const {
    return CellStatus::kFree;
  }
  virtual CellStatus getLineStatusBoundingBox(
      const Eigen::Vector3d& start, const Eigen::Vector3d& end,
      const Eigen::Vector3d& bounding_box_size) const {
    return CellStatus::kFree;
  }

  virtual void getOccupiedPointCloud(
      pcl::PointCloud<pcl::PointXYZ> *output_cloud) const {
    // Blank world by default, so don't fill the pointcloud.
    return;
  }

  virtual void getOccupiedPointcloudInBoundingBox(
      const Eigen::Vector3d& center, const Eigen::Vector3d& bounding_box_size,
      pcl::PointCloud<pcl::PointXYZ>* output_cloud) const {
    // Blank world by default, so don't fill the pointcloud.
    return;
  }

  // Collision checking with a robot model.
  virtual void setRobotSize(double x, double y, double z) {
    setRobotSize(Eigen::Vector3d(x, y, z));
  }
  virtual void setRobotSize(const Eigen::Vector3d& robot_size) { return; }
  virtual Eigen::Vector3d getRobotSize() const {
    return Eigen::Vector3d::Zero();
  }

  virtual bool checkCollisionWithRobot(const Eigen::Vector3d& robot_position) {
    return false;
  }
  // Checks a path (assumed to be time-ordered) for collision.
  // Sets the second input to the index at which the collision occurred.
  virtual bool checkPathForCollisionsWithRobot(
      const std::vector<Eigen::Vector3d>& robot_positions,
      size_t* collision_index) {
    return false;
  }

  virtual Eigen::Vector3d getMapCenter() const {
    return Eigen::Vector3d::Zero();
  }
  virtual Eigen::Vector3d getMapSize() const {
    return Eigen::Vector3d(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
  }
  virtual void getMapBounds(Eigen::Vector3d* min_bound,
                            Eigen::Vector3d* max_bound) const {
    CHECK_NOTNULL(min_bound);
    CHECK_NOTNULL(max_bound);
    *min_bound = Eigen::Vector3d(std::numeric_limits<double>::min(),
                                 std::numeric_limits<double>::min(),
                                 std::numeric_limits<double>::min());
    *max_bound = Eigen::Vector3d(std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max(),
                                 std::numeric_limits<double>::max());
  }

  // Weighing class for points -> affect the weight of each point inserted
  // into the map.
  // If the weighing class is set, the "with weights" version if used for
  // all insertion functions.
  void setPointWeighing(const std::shared_ptr<PointWeighing>& point_weighing) {
    point_weighing_ = point_weighing;
  }

  void clearPointWeighing() { point_weighing_.reset(); }

  bool isPointWeighingSet() const {
    if (point_weighing_) {
      return true;
    }
    return false;
  }

 protected:
  // Called by insertDisparityImage(). Inheriting classes need to implement
  // this.
  // Input is the sensor to world transform and projected points in 3D in
  // the sensor coordinate frame, of type CV_32FC3.
  virtual void insertProjectedDisparityIntoMapImpl(
      const Transformation& sensor_to_world, const cv::Mat& projected_points) {
    LOG(ERROR) << "Calling unimplemented disparity insertion!";
  }
  virtual void insertProjectedDisparityIntoMapWithWeightsImpl(
      const Transformation& sensor_to_world, const cv::Mat& projected_points,
      const cv::Mat& weights) {
    LOG(ERROR) << "Calling unimplemented disparity insertion!";
  }

  virtual void insertPointcloudIntoMapImpl(
      const Transformation& T_G_sensor,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_sensor) {
    LOG(ERROR) << "Calling unimplemented pointcloud insertion!";
  }
  virtual void insertPointcloudIntoMapWithWeightsImpl(
      const Transformation& sensor_to_world,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
      const std::vector<double>& weights) {
    LOG(ERROR) << "Calling unimplemented disparity insertion!";
  }

  // Generate Q matrix from parameters.
  Eigen::Matrix4d generateQ(double Tx, double left_cx, double left_cy,
                            double left_fx, double left_fy, double right_cx,
                            double right_cy, double right_fx,
                            double right_fy) const;

  // Compute weights from disparities.
  void computeWeights(const cv::Mat& disparity, cv::Mat* weights) const;

  // Compute weights from pointcloud data.
  void computeWeights(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      std::vector<double>* weights) const;

  std::shared_ptr<PointWeighing> point_weighing_;
};

}  // namespace volumetric_mapping
#endif  // VOLUMETRIC_MAP_BASE_WORLD_BASE_H_
