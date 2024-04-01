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

#include "volumetric_map_base/world_base.h"

#ifndef COMPILE_WITHOUT_ROS
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#endif

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/conversions.h>
//#include <pcl_conversions/pcl_conversions.h>

namespace volumetric_mapping {

#ifndef COMPILE_WITHOUT_ROS
void WorldBase::insertDisparityImage(
    const Transformation& sensor_to_world,
    const stereo_msgs::DisparityImageConstPtr& disparity,
    const Eigen::Matrix4d& Q_full, const Eigen::Vector2d& full_image_size) {
  cv_bridge::CvImageConstPtr cv_img_ptr =
      cv_bridge::toCvShare(disparity->image, disparity);
  insertDisparityImage(sensor_to_world, cv_img_ptr->image, Q_full,
                       full_image_size);
}
#endif

void WorldBase::insertDisparityImage(const Transformation& sensor_to_world,
                                     const cv::Mat& disparity,
                                     const Eigen::Matrix4d& Q_full,
                                     const Eigen::Vector2d& full_image_size) {
  // Figure out the downsampling of the image.
  double downsampling_factor = full_image_size.x() / disparity.cols;
  Eigen::Matrix4d Q = Q_full;
  if (fabs(downsampling_factor - 1.0) > 1e-6) {
    // c{x,y} and f{x,y} are scaled by the downsampling factor then.
    const double downsampling_squared =
        downsampling_factor * downsampling_factor;
    Q(0, 0) /= downsampling_factor;
    Q(0, 3) /= downsampling_squared;
    Q(1, 1) /= downsampling_factor;
    Q(1, 3) /= downsampling_squared;
    Q(2, 3) /= downsampling_squared;
    Q(3, 2) /= downsampling_factor;
    Q(3, 3) /= downsampling_factor;
  }

  cv::Mat reprojected_disparities(disparity.size(), CV_32FC3);
  cv::Mat Q_cv;
  cv::eigen2cv(Q, Q_cv);

  cv::reprojectImageTo3D(disparity, reprojected_disparities, Q_cv, true);

  // Call the implementation function of the inheriting class.
  if (!isPointWeighingSet()) {
    insertProjectedDisparityIntoMapImpl(sensor_to_world,
                                        reprojected_disparities);
  } else {
    cv::Mat weights;
    computeWeights(disparity, &weights);
    insertProjectedDisparityIntoMapWithWeightsImpl(
        sensor_to_world, reprojected_disparities, weights);
  }
}

// Helper functions to compute the Q matrix for given UNRECTIFIED camera
// parameters.
// Assumes 0 distortion.
Eigen::Matrix4d WorldBase::getQForCameras(
    const Transformation& T_C1_C0, const Eigen::Matrix3d& left_cam_matrix,
    const Eigen::Matrix3d& right_cam_matrix,
    const Eigen::Vector2d& full_image_size) const {
  // So unfortunately... Have to actually stereo rectify this pair.
  // Input matrices: C = camera matrix (3x3), D = disortion coefficients (5x1),
  // R = rotation matrix between cameras (3x3), T = translation between cameras
  // (3x1).
  cv::Mat C0, D0, C1, D1, R, T;
  // For now, ignore distortion coefficients.
  Eigen::Matrix<double, 5, 1> distortion = Eigen::Matrix<double, 5, 1>::Zero();

  // Output parameters.
  cv::Mat R0, R1, P0, P1, Q_cv;

  cv::eigen2cv(left_cam_matrix, C0);
  cv::eigen2cv(right_cam_matrix, C1);
  cv::eigen2cv(distortion, D0);
  cv::eigen2cv(distortion, D1);
  cv::eigen2cv(T_C1_C0.getRotationMatrix(), R);
  cv::eigen2cv(T_C1_C0.getPosition(), T);
  cv::Size img_size(full_image_size.x(), full_image_size.y());

  cv::Rect roi1, roi2;
  // Stereo rectify:
  cv::stereoRectify(C0, D0, C1, D1, img_size, R, T, R0, R1, P0, P1, Q_cv,
                    cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2);

  Eigen::Matrix4d Q;
  cv::cv2eigen(Q_cv, Q);
  return Q;
}

#ifndef COMPILE_WITHOUT_ROS
Eigen::Matrix4d WorldBase::getQForROSCameras(
    const sensor_msgs::CameraInfo& left_camera,
    const sensor_msgs::CameraInfo& right_camera) const {
  // Unfortunately updateQ is protected in StereoCameraModel.
  image_geometry::StereoCameraModel stereo_model;
  stereo_model.fromCameraInfo(left_camera, right_camera);

  double Tx = -stereo_model.baseline();
  double left_cx = stereo_model.left().cx();
  double left_cy = stereo_model.left().cy();
  double left_fx = stereo_model.left().fx();
  double left_fy = stereo_model.left().fy();
  double right_cx = stereo_model.right().cx();
  double right_cy = stereo_model.right().cy();
  double right_fx = stereo_model.right().fx();
  double right_fy = stereo_model.right().fy();

  return generateQ(Tx, left_cx, left_cy, left_fx, left_fy, right_cx, right_cy,
                   right_fx, right_fy);
}
#endif

Eigen::Matrix4d WorldBase::generateQ(double Tx, double left_cx, double left_cy,
                                     double left_fx, double left_fy,
                                     double right_cx, double right_cy,
                                     double right_fx, double right_fy) const {
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

  // Basically do the same that the stereo model does by hand:
  // See: https://github.com/ros-perception/vision_opencv/blob/indigo/
  //      image_geometry/src/stereo_camera_model.cpp#L53
  //
  //  From Springer Handbook of Robotics, p. 524:
  // [x y z 1]^T = Q * [u v d 1]^T
  // Where Q is defined as
  // Q = [ FyTx  0     0   -FyCxTx     ]
  //     [ 0     FxTx  0   -FxCyTx     ]
  //     [ 0     0     0    FxFyTx     ]
  //     [ 0     0     -Fy  Fy(Cx-Cx') ]
  //  where primed parameters are from the left projection matrix,
  //  unprimed from the right.
  // Helen's note: their actual implementation uses the left cam as unprimed,
  // which should make more sense since we always have left disparities anyway.
  //  Disparity = x_left - x_right

  Q(0, 0) = left_fy * Tx;
  Q(0, 3) = -left_fy * left_cx * Tx;
  Q(1, 1) = left_fx * Tx;
  Q(1, 3) = -left_fx * left_cy * Tx;
  Q(2, 3) = left_fx * left_fy * Tx;
  Q(3, 2) = -left_fy;
  // Zero when disparities are pre-adjusted for the difference in projection
  // centers of the 2 cameras.
  Q(3, 3) = left_fy * (left_cx - right_cx);

  return Q;
}

#ifndef COMPILE_WITHOUT_ROS
void WorldBase::insertPointcloud(
    const Transformation& T_G_sensor,
    const sensor_msgs::PointCloud2::ConstPtr& pointcloud_sensor) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_sensor_pcl(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloud_sensor, *pointcloud_sensor_pcl);
  insertPointcloud(T_G_sensor, pointcloud_sensor_pcl);
}
#endif

// TODO(tcies) Make the virtual function insertPointcloudIntoMapImpl take a
// signature like this so that the transformation logic doesn't need to be
// repeated in all derived classes.
void WorldBase::insertPointcloud(const Transformation& T_G_sensor,
                                 const Eigen::Matrix3Xd& pointcloud_sensor) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_sensor_pcl(
      new pcl::PointCloud<pcl::PointXYZ>(pointcloud_sensor.cols(), 1));
  pointcloud_sensor_pcl->getMatrixXfMap() = pointcloud_sensor.cast<float>();
  insertPointcloud(T_G_sensor, pointcloud_sensor_pcl);
}

void WorldBase::insertPointcloud(
    const Transformation& T_G_sensor,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_sensor) {
  if (!isPointWeighingSet()) {
    insertPointcloudIntoMapImpl(T_G_sensor, pointcloud_sensor);
  } else {
    std::vector<double> weights;
    computeWeights(pointcloud_sensor, &weights);
    insertPointcloudIntoMapWithWeightsImpl(T_G_sensor, pointcloud_sensor,
                                           weights);
  }
}

void WorldBase::computeWeights(const cv::Mat& disparity,
                               cv::Mat* weights) const {
  *weights = cv::Mat::ones(disparity.rows, disparity.cols, CV_32F);

  if (!point_weighing_) {
    return;
  }

  for (int v = 0; v < disparity.rows; ++v) {
    // Disparity is 32f.
    const float* row_pointer = disparity.ptr<float>(v);
    for (int u = 0; u < disparity.cols; ++u) {
      weights->at<float>(v, u) =
          point_weighing_->computeWeightForDisparity(u, v, row_pointer[u]);
    }
  }
}

void WorldBase::computeWeights(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               std::vector<double>* weights) const {
  weights->resize(cloud->size(), 1.0);

  if (!point_weighing_) {
    return;
  }

  unsigned int index = 0;
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin();
       it != cloud->end(); ++it) {
    (*weights)[index] =
        point_weighing_->computeWeightForPoint(it->x, it->y, it->z);
    index++;
  }
}

}  // namespace volumetric_mapping
