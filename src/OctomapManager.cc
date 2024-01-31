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

#include "OctomapManager.h"

#include <glog/logging.h>

//#include <minkindr_conversions/kindr_tf.h>
//#include <minkindr_conversions/kindr_msg.h>
//#include <minkindr_conversions/kindr_xml.h>

#include "PointDefinitions.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace PLVS2
{



using namespace volumetric_mapping;

OctomapManager::OctomapManager()
{
    OctomapParameters params;
    // Set the parent class parameters.
    setOctomapParameters(params);
    
//    setParametersFromROS();
//    subscribe();
//    advertiseServices();
//    advertisePublishers();

    // After creating the manager, if the octomap_file parameter is set,
    // load the octomap at that path and publish it.
//    std::string octomap_file;
//    if (nh_private_.getParam("octomap_file", octomap_file))
//    {
//        if (loadOctomapFromFile(octomap_file))
//        {
//            ROS_INFO_STREAM(
//                            "Successfully loaded octomap from path: " << octomap_file);
//            publishAll();
//        }
//        else
//        {
//            ROS_ERROR_STREAM("Could not load octomap from path: " << octomap_file);
//        }
//    }
}
//
//void OctomapManager::setParametersFromROS()
//{
//    OctomapParameters params;
//    nh_private_.param("tf_frame", world_frame_, world_frame_);
//    nh_private_.param("robot_frame", robot_frame_, robot_frame_);
//    nh_private_.param("resolution", params.resolution, params.resolution);
//    nh_private_.param("probability_hit", params.probability_hit,
//                      params.probability_hit);
//    nh_private_.param("probability_miss", params.probability_miss,
//                      params.probability_miss);
//    nh_private_.param("threshold_min", params.threshold_min,
//                      params.threshold_min);
//    nh_private_.param("threshold_max", params.threshold_max,
//                      params.threshold_max);
//    nh_private_.param("threshold_occupancy", params.threshold_occupancy,
//                      params.threshold_occupancy);
//    nh_private_.param("filter_speckles", params.filter_speckles,
//                      params.filter_speckles);
//    nh_private_.param("max_free_space", params.max_free_space,
//                      params.max_free_space);
//    nh_private_.param("min_height_free_space", params.min_height_free_space,
//                      params.min_height_free_space);
//    nh_private_.param("sensor_max_range", params.sensor_max_range,
//                      params.sensor_max_range);
//    nh_private_.param("visualize_min_z", params.visualize_min_z,
//                      params.visualize_min_z);
//    nh_private_.param("visualize_max_z", params.visualize_max_z,
//                      params.visualize_max_z);
//    nh_private_.param("full_image_width", full_image_size_.x(),
//                      full_image_size_.x());
//    nh_private_.param("full_image_height", full_image_size_.y(),
//                      full_image_size_.y());
//    nh_private_.param("map_publish_frequency", map_publish_frequency_,
//                      map_publish_frequency_);
//    nh_private_.param("treat_unknown_as_occupied",
//                      params.treat_unknown_as_occupied,
//                      params.treat_unknown_as_occupied);
//    nh_private_.param("change_detection_enabled", params.change_detection_enabled,
//                      params.change_detection_enabled);
//
//    // Try to initialize Q matrix from parameters, if available.
//    std::vector<double> Q_vec;
//    if (nh_private_.getParam("Q", Q_vec))
//    {
//        Q_initialized_ = setQFromParams(&Q_vec);
//    }
//
//    // Publisher/subscriber settings.
//    nh_private_.param("latch_topics", latch_topics_, latch_topics_);
//
//    // Transform settings.
//    nh_private_.param("use_tf_transforms", use_tf_transforms_,
//                      use_tf_transforms_);
//    // If we use topic transforms, we have 2 parts: a dynamic transform from a
//    // topic and a static transform from parameters.
//    // Static transform should be T_G_D (where D is whatever sensor the
//    // dynamic coordinate frame is in) and the static should be T_D_C (where
//    // C is the sensor frame that produces the depth data). It is possible to
//    // specific T_C_D and set invert_static_tranform to true.
//    if (!use_tf_transforms_)
//    {
//        transform_sub_ = nh_.subscribe("transform", 40,
//                                       &OctomapManager::transformCallback, this);
//        // Retrieve T_D_C from params.
//        XmlRpc::XmlRpcValue T_B_D_xml;
//        // TODO(helenol): split out into a function to avoid duplication.
//        if (nh_private_.getParam("T_B_D", T_B_D_xml))
//        {
//            kindr::minimal::xmlRpcToKindr(T_B_D_xml, &T_B_D_);
//
//            // See if we need to invert it.
//            bool invert_static_tranform = false;
//            nh_private_.param("invert_T_B_D", invert_static_tranform,
//                              invert_static_tranform);
//            if (invert_static_tranform)
//            {
//                T_B_D_ = T_B_D_.inverse();
//            }
//        }
//        XmlRpc::XmlRpcValue T_B_C_xml;
//        if (nh_private_.getParam("T_B_C", T_B_C_xml))
//        {
//            kindr::minimal::xmlRpcToKindr(T_B_C_xml, &T_B_C_);
//
//            // See if we need to invert it.
//            bool invert_static_tranform = false;
//            nh_private_.param("invert_T_B_C", invert_static_tranform,
//                              invert_static_tranform);
//            if (invert_static_tranform)
//            {
//                T_B_C_ = T_B_C_.inverse();
//            }
//        }
//    }
//
//    // Set the parent class parameters.
//    setOctomapParameters(params);
//}

//bool OctomapManager::setQFromParams(std::vector<double>* Q_vec)
//{
//    if (Q_vec->size() != 16)
//    {
//        std::cout << "Invalid Q matrix size, expected size: 16, actual size: " << Q_vec->size() << std::endl; 
//        return false;
//    } 
//
//    // Try to map the vector as coefficients.
//    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > Q_vec_map(
//                                                                        Q_vec->data());
//    // Copy over to the Q member.
//    Q_ = Q_vec_map;
//
//    return true;
//}

//void OctomapManager::subscribe()
//{
//    left_info_sub_ = nh_.subscribe("cam0/camera_info", 1,
//                                   &OctomapManager::leftCameraInfoCallback, this);
//    right_info_sub_ = nh_.subscribe(
//                                    "cam1/camera_info", 1, &OctomapManager::rightCameraInfoCallback, this);
//    disparity_sub_ = nh_.subscribe(
//                                   "disparity", 40, &OctomapManager::insertDisparityImageWithTf, this);
//    pointcloud_sub_ = nh_.subscribe(
//                                    "pointcloud", 40, &OctomapManager::insertPointcloudWithTf, this);
//    octomap_sub_ =
//            nh_.subscribe("input_octomap", 1, &OctomapManager::octomapCallback, this);
//}

//void OctomapManager::octomapCallback(const octomap_msgs::Octomap& msg)
//{
//    setOctomapFromMsg(msg);
//    publishAll();
//    ROS_INFO_ONCE("Got octomap from message.");
//}

//void OctomapManager::advertiseServices()
//{
//    reset_map_service_ = nh_private_.advertiseService(
//                                                      "reset_map", &OctomapManager::resetMapCallback, this);
//    publish_all_service_ = nh_private_.advertiseService(
//                                                        "publish_all", &OctomapManager::publishAllCallback, this);
//    get_map_service_ = nh_private_.advertiseService(
//                                                    "get_map", &OctomapManager::getOctomapCallback, this);
//    save_octree_service_ = nh_private_.advertiseService(
//                                                        "save_map", &OctomapManager::saveOctomapCallback, this);
//    load_octree_service_ = nh_private_.advertiseService(
//                                                        "load_map", &OctomapManager::loadOctomapCallback, this);
//    save_point_cloud_service_ = nh_private_.advertiseService(
//                                                             "save_point_cloud", &OctomapManager::savePointCloudCallback, this);
//    set_box_occupancy_service_ = nh_private_.advertiseService(
//                                                              "set_box_occupancy", &OctomapManager::setBoxOccupancyCallback, this);
//    set_display_bounds_service_ = nh_private_.advertiseService(
//                                                               "set_display_bounds", &OctomapManager::setDisplayBoundsCallback, this);
//}

//void OctomapManager::advertisePublishers()
//{
//    occupied_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
//            "octomap_occupied", 1, latch_topics_);
//    free_nodes_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
//            "octomap_free", 1, latch_topics_);
//
//    binary_map_pub_ = nh_private_.advertise<octomap_msgs::Octomap>(
//            "octomap_binary", 1, latch_topics_);
//    full_map_pub_ = nh_private_.advertise<octomap_msgs::Octomap>(
//            "octomap_full", 1, latch_topics_);
//
//    pcl_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>("octomap_pcl", 1,
//            latch_topics_);
//    nearest_obstacle_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
//            "nearest_obstacle", 1, false);
//
//    if (map_publish_frequency_ > 0.0)
//    {
//        map_publish_timer_ =
//                nh_private_.createTimer(ros::Duration(1.0 / map_publish_frequency_),
//                                        &OctomapManager::publishAllEvent, this);
//    }
//}

//void OctomapManager::publishAll()
//{
//    if (latch_topics_ || occupied_nodes_pub_.getNumSubscribers() > 0 ||
//            free_nodes_pub_.getNumSubscribers() > 0)
//    {
//        visualization_msgs::MarkerArray occupied_nodes, free_nodes;
//        generateMarkerArray(world_frame_, &occupied_nodes, &free_nodes);
//        occupied_nodes_pub_.publish(occupied_nodes);
//        free_nodes_pub_.publish(free_nodes);
//    }
//
//    if (latch_topics_ || binary_map_pub_.getNumSubscribers() > 0)
//    {
//        octomap_msgs::Octomap binary_map;
//        getOctomapBinaryMsg(&binary_map);
//        binary_map.header.frame_id = world_frame_;
//        binary_map_pub_.publish(binary_map);
//    }
//
//    if (latch_topics_ || full_map_pub_.getNumSubscribers() > 0)
//    {
//        octomap_msgs::Octomap full_map;
//        getOctomapBinaryMsg(&full_map);
//        full_map.header.frame_id = world_frame_;
//        full_map_pub_.publish(full_map);
//    }
//
//    if (latch_topics_ || pcl_pub_.getNumSubscribers() > 0)
//    {
//        pcl::PointCloud<pcl::PointXYZ> point_cloud;
//        getOccupiedPointCloud(&point_cloud);
//        sensor_msgs::PointCloud2 cloud;
//        pcl::toROSMsg(point_cloud, cloud);
//        cloud.header.frame_id = world_frame_;
//        pcl_pub_.publish(cloud);
//    }
//
//    if (use_tf_transforms_ && nearest_obstacle_pub_.getNumSubscribers() > 0)
//    {
//        Transformation robot_to_world;
//        if (lookupTransformTf(robot_frame_, world_frame_, ros::Time::now(),
//                              &robot_to_world))
//        {
//            Eigen::Vector3d robot_center = robot_to_world.getPosition();
//            pcl::PointCloud<pcl::PointXYZ> point_cloud;
//            getOccupiedPointcloudInBoundingBox(robot_center, robot_size_, &point_cloud);
//            sensor_msgs::PointCloud2 cloud;
//            pcl::toROSMsg(point_cloud, cloud);
//            cloud.header.frame_id = world_frame_;
//            cloud.header.stamp = ros::Time::now();
//            nearest_obstacle_pub_.publish(cloud);
//        }
//    }
//}

//void OctomapManager::publishAllEvent(const ros::TimerEvent& e)
//{
//    publishAll();
//}

//bool OctomapManager::resetMapCallback(std_srvs::Empty::Request& request,
//                                      std_srvs::Empty::Response& response)
//{
//    resetMap();
//    return true;
//}

//bool OctomapManager::publishAllCallback(std_srvs::Empty::Request& request,
//                                        std_srvs::Empty::Response& response)
//{
//    publishAll();
//    return true;
//}

//bool OctomapManager::getOctomapCallback(
//                                        octomap_msgs::GetOctomap::Request& request,
//                                        octomap_msgs::GetOctomap::Response& response)
//{
//    return getOctomapFullMsg(&response.map);
//}


bool OctomapManager::loadOctomap(const std::string& file_path)
{
    std::string extension = file_path.substr(file_path.find_last_of(".") + 1);
    if (extension == "bt")
    {
        return loadOctomapFromFile(file_path);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                                                  new pcl::PointCloud<pcl::PointXYZ>);
        if (extension == "pcd")
        {
            pcl::io::loadPCDFile < pcl::PointXYZ > (file_path, *cloud);
        }
        else if (extension == "ply")
        {
            pcl::io::loadPLYFile < pcl::PointXYZ > (file_path, *cloud);
        }
        else
        {
            std::cout << "No known file extension (.bt, .pcd, .ply): " << file_path << std::endl;
            return false;
        }
        octomap::KeySet free_cells, occupied_cells;
        for (size_t i = 0u; i < cloud->size(); ++i)
        {
            const octomap::point3d p_G_point((*cloud)[i].x, (*cloud)[i].y,
                                             (*cloud)[i].z);
            octomap::OcTreeKey key;
            if (octree_->coordToKeyChecked(p_G_point, key))
            {
                occupied_cells.insert(key);
            }
        }
        updateOccupancy(&free_cells, &occupied_cells);
        return true;
    }
}

bool OctomapManager::saveOctomap(const std::string& file_path)
{
    return writeOctomapToFile(file_path);
}

bool OctomapManager::savePointCloud(const std::string& file_path)
{
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    getOccupiedPointCloud(&point_cloud);
    pcl::io::savePLYFileASCII(file_path, point_cloud);
    return true;
}

//bool OctomapManager::setBoxOccupancyCallback(
//                                             volumetric_msgs::SetBoxOccupancy::Request& request,
//                                             volumetric_msgs::SetBoxOccupancy::Response& response)
//{
//    Eigen::Vector3d bounding_box_center;
//    Eigen::Vector3d bounding_box_size;
//
//    tf::vectorMsgToKindr(request.box_center, &bounding_box_center);
//    tf::vectorMsgToKindr(request.box_size, &bounding_box_size);
//    bool set_occupied = request.set_occupied;
//
//    if (set_occupied)
//    {
//        setOccupied(bounding_box_center, bounding_box_size);
//    }
//    else
//    {
//        setFree(bounding_box_center, bounding_box_size);
//    }
//    publishAll();
//    return true;
//}

//bool OctomapManager::setDisplayBoundsCallback(
//                                              volumetric_msgs::SetDisplayBounds::Request& request,
//                                              volumetric_msgs::SetDisplayBounds::Response& response)
//{
//    params_.visualize_min_z = request.min_z;
//    params_.visualize_max_z = request.max_z;
//    publishAll();
//    return true;
//}

//void OctomapManager::leftCameraInfoCallback(
//                                            const sensor_msgs::CameraInfoPtr& left_info)
//{
//    left_info_ = left_info;
//    if (left_info_ && right_info_ && !Q_initialized_)
//    {
//        calculateQ();
//    }
//}
//
//void OctomapManager::rightCameraInfoCallback(
//                                             const sensor_msgs::CameraInfoPtr& right_info)
//{
//    right_info_ = right_info;
//    if (left_info_ && right_info_ && !Q_initialized_)
//    {
//        calculateQ();
//    }
//}

//void OctomapManager::calculateQ()
//{
//    Q_ = getQForROSCameras(*left_info_, *right_info_);
//    full_image_size_.x() = left_info_->width;
//    full_image_size_.y() = left_info_->height;
//    Q_initialized_ = true;
//}

//void OctomapManager::insertDisparityImageWithTf(
//                                                const stereo_msgs::DisparityImageConstPtr& disparity)
//{
//    if (!Q_initialized_)
//    {
//        ROS_WARN_THROTTLE(
//                          1, "No camera info available yet, skipping adding disparity.");
//        return;
//    }
//
//    // Look up transform from sensor frame to world frame.
//    Transformation sensor_to_world;
//    if (lookupTransform(disparity->header.frame_id, world_frame_,
//                        disparity->header.stamp, &sensor_to_world))
//    {
//        insertDisparityImage(sensor_to_world, disparity, Q_, full_image_size_);
//    }
//}

//void OctomapManager::insertPointcloudWithTf(
//                                            const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
//{
//    // Look up transform from sensor frame to world frame.
//    Transformation sensor_to_world;
//    if (lookupTransform(pointcloud->header.frame_id, world_frame_,
//                        pointcloud->header.stamp, &sensor_to_world))
//    {
//        insertPointcloud(sensor_to_world, pointcloud);
//    }
//}

void OctomapManager::insertPointcloud(const volumetric_mapping::Transformation& T_G_sensor, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_sensor)
{
    insertPointcloud(T_G_sensor, pointcloud_sensor);
}




} // namespace volumetric_mapping
