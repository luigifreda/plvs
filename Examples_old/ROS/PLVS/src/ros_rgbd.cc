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
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <../../../include/PointDefinitions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <../../../include/System.h>
#include <../../../include/PointCloudMapping.h>
#include <../../../include/Tracking.h>
#include <../../../include/Utils.h>
#include <../../../include/Logger.h>

#include <../../../Thirdparty/Sophus/sophus/geometry.hpp>


// Set to true if you want a direct transformation from Optical Frame to World
#define T_WORLD_OPTICAL true

using namespace std;

bool bRGB = true;
bool bUseViewer = true;

std::shared_ptr<PLVS2::System> pSLAM;

std::shared_ptr<PLVS2::PointCloudMapping> pPointCloudMapping;
ros::Publisher pointCloudPublisher;
std::uint64_t map_timestamp = 0;

int trackingState = (int)PLVS2::Tracking::NOT_INITIALIZED;

std::string camera_slam_frame_id = "camera_link_orbslam";

bool bWaitForCameraInfo = true;
std::mutex mMutexGotCameraInfo;
std::atomic_bool bGotCameraInfo = false;
int nTimeOutWaitCameraInfoMs = 10000; // time out for waiting camera info [ms]

float baseline = 1.;

class ImageGrabber
{
public:
    ImageGrabber(PLVS2::System* pSLAM):mpSLAM(pSLAM){}

    ImageGrabber() : b_was_lost(false)
    {
        // T_wo = cv::Mat::eye(4, 4, CV_32F);
        // T_wl = cv::Mat::eye(4, 4, CV_32F);

        // T_ol = (cv::Mat_<float>(4, 4) << 0, -1, 0, 0,
        //         0, 0, -1, 0,
        //         1, 0, 0, 0,
        //         0, 0, 0, 1);
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << 0, -1, 0,  0, 0, -1,  1, 0, 0;
        Eigen::Vector3f translation;
        translation << 0, 0, 0;
        T_ol = Sophus::SE3f(rotation_matrix, translation);

        // T_wo_zero = cv::Mat::eye(4, 4, CV_32F);
    }

    void SetSlamSystem(PLVS2::System* pSLAM)
    {
        mpSLAM = pSLAM;
    }

    void SendInitialMapTransform();
    void SendTransform(const ros::Time& time);

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

protected:

    PLVS2::System* mpSLAM;
    bool b_was_lost;

    // w = WORLD
    // o = OPTICAL FRAME
    // l = LINK FRAME
    tf::TransformBroadcaster br;
    Sophus::SE3f T_wo_zero;
    Sophus::SE3f T_wo;
    Sophus::SE3f T_wl;
    Sophus::SE3f T_ol;

    std::mutex grab_mutex;
};

// TODO: to re-enable
#define USE_POINTCLOUD_MAPPING 0
#if USE_POINTCLOUD_MAPPING
void pointcloudCallback(const ros::TimerEvent& event)
{
    if (pPointCloudMapping)
    {
        std::uint64_t current_timestamp = pPointCloudMapping->GetMapTimestamp();
        if ( (pointCloudPublisher.getNumSubscribers() > 0) && (current_timestamp > map_timestamp) )
        {
            map_timestamp = current_timestamp;
            PLVS2::PointCloudMapping::PointCloudT::Ptr pMap = pPointCloudMapping->GetMap();
            if (!pMap) return; /// < EXIT POINT
            pMap->header.frame_id = "map";
            sensor_msgs::PointCloud2 map_msg;
            pcl::PointCloud<pcl::PointXYZRGB>* pMapRGB = (pcl::PointCloud<pcl::PointXYZRGB>*)(pMap.get());
            pcl::toROSMsg(*pMapRGB, map_msg);
            pointCloudPublisher.publish(map_msg);
            ROS_INFO_STREAM("*** map published ***");
        }
    }
}
#endif 

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    unique_lock<mutex> lock(mMutexGotCameraInfo);
    if(!bGotCameraInfo)
    {        
        std::cout << "camera info has been received and set" << std::endl;
        
        // 3x3 row-major matrix
        //     [fx  0 cx]
        // K = [ 0 fy cy]
        //     [ 0  0  1]        
        const float fx = msg->K[0]; const float cx = msg->K[2];
        const float fy = msg->K[4]; const float cy = msg->K[5];
        
        std:stringstream ss;
        ss << std::fixed << std::setprecision(8);
        
        ss << "fx: " << fx << std::endl;
        ss << "fy: " << fy << std::endl;
        ss << "cx: " << cx << std::endl;
        ss << "cy: " << cy << std::endl;           
               
        const float bf = baseline * fx;
        ss << "bf: " << bf << std::endl;      

        // For "plumb_bob", D = [k1, k2, t1, t2, k3].        
        cv::Mat distCoef(5,1,CV_32F);
        distCoef.at<float>(0) = msg->D[0];
        distCoef.at<float>(1) = msg->D[1];
        distCoef.at<float>(2) = msg->D[2];
        distCoef.at<float>(3) = msg->D[3];
        distCoef.at<float>(4) = msg->D[4];
        ss << "distCoef: " << distCoef << std::endl;      
        
        
        pSLAM->SetCalibration(fx, fy, cx, cy, distCoef, bf);
        
        Logger logger("receivedCalibration.txt");
        logger << ss.str();      
        
        bGotCameraInfo = true;         
    }
}

void waitCameraInfoTimerCallback(const ros::TimerEvent&)
{
    unique_lock<mutex> lock(mMutexGotCameraInfo);    
    if(!bGotCameraInfo)
    {
        std::cout << "camera info was not received" << std::endl; 
        Logger logger("receivedCalibration.txt");
        logger << "none - used yml file";             
        bGotCameraInfo = true;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun PLVS RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool> ((int) fSettings["Camera.RGB"]);
    bUseViewer = static_cast<int> (PLVS2::Utils::GetParam(fSettings, "Viewer.on", 1)) != 0;
    bWaitForCameraInfo = static_cast<int> (PLVS2::Utils::GetParam(fSettings, "Camera.waitCameraInfoOn", 0)) != 0;
    
    baseline = static_cast<float>(fSettings["Camera.bf"])/static_cast<float>(fSettings["Camera.fx"]);

    ImageGrabber igb;
    igb.SendInitialMapTransform();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //PLVS2::System SLAM(argv[1], argv[2], PLVS2::System::RGBD, bUseViewer);
    pSLAM = std::make_shared<PLVS2::System >(argv[1], argv[2], PLVS2::System::RGBD, bUseViewer);

    igb.SetSlamSystem(pSLAM.get());

    ros::NodeHandle nh;

    const int subscriber_queue_size=1; // real-time 1

    /// < TODO: use only rectified color images (otherwise we have to manage the undistortion inside the point cloud mapping)
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", subscriber_queue_size);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));
    
    ros::Subscriber info_color_sub;
    ros::Timer timerWaitCameraInfo;
    if(bWaitForCameraInfo)
    {
        bGotCameraInfo = false;
        info_color_sub = nh.subscribe("/camera/rgb/camera_info", 1, cameraInfoCallback);
        std::cout << "waiting for camera info" << std::endl;
        timerWaitCameraInfo = nh.createTimer(ros::Duration(nTimeOutWaitCameraInfoMs*0.001), waitCameraInfoTimerCallback, /*oneshot*/true);
    }
    else
    {
        bGotCameraInfo = true;
    }

    ros::Timer timer;
    if (!bUseViewer)
    {
        pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/orbslam2/pointcloud_map", 1, true);
#if USE_POINTCLOUD_MAPPING
        pPointCloudMapping = pSLAM->GetPointCloudMapping();
        timer = nh.createTimer(ros::Duration(1.0), pointcloudCallback);
#endif 
    }

#if 1
    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
#else
    ros::spin();
#endif

    // Stop all threads
    pSLAM->Shutdown();

    // Save camera trajectory
    pSLAM->SaveTrajectoryTUM("CameraTrajectory.txt");
    pSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::SendInitialMapTransform()
{
    T_wl = T_wo*T_ol;

    SendTransform(ros::Time::now());
}

void ImageGrabber::SendTransform(const ros::Time& time)
{
    //std::cout << "SendTransform()" << std::endl;

#if T_WORLD_OPTICAL

    Sophus::SE3f::Transformation T_wo_mat = T_wo.matrix(); 
    // tf::Matrix3x3 cameraRotation(T_wo.at<float>(0, 0), T_wo.at<float>(0, 1), T_wo.at<float>(0, 2),
    //                              T_wo.at<float>(1, 0), T_wo.at<float>(1, 1), T_wo.at<float>(1, 2),
    //                              T_wo.at<float>(2, 0), T_wo.at<float>(2, 1), T_wo.at<float>(2, 2));
    tf::Matrix3x3 cameraRotation(T_wo_mat(0, 0), T_wo_mat(0, 1), T_wo_mat(0, 2),
                                 T_wo_mat(1, 0), T_wo_mat(1, 1), T_wo_mat(1, 2),
                                 T_wo_mat(2, 0), T_wo_mat(2, 1), T_wo_mat(2, 2));                                 

    tf::Vector3 cameraTranslation(T_wo_mat(0, 3), T_wo_mat(1, 3), T_wo_mat(2, 3));

    tf::Transform T_WorldOptical = tf::Transform(cameraRotation, cameraTranslation);
    br.sendTransform(tf::StampedTransform(T_WorldOptical, ros::Time::now(), "map", camera_slam_frame_id));

#else

    tf::Matrix3x3 cameraRotation(T_wl.at<float>(0, 0), T_wl.at<float>(0, 1), T_wl.at<float>(0, 2),
                                 T_wl.at<float>(1, 0), T_wl.at<float>(1, 1), T_wl.at<float>(1, 2),
                                 T_wl.at<float>(2, 0), T_wl.at<float>(2, 1), T_wl.at<float>(2, 2));

    tf::Vector3 cameraTranslation(T_wl.at<float>(0, 3), T_wl.at<float>(1, 3), T_wl.at<float>(2, 3));

    tf::Transform T_WorldLink = tf::Transform(cameraRotation, cameraTranslation);
    //StampedTransform (const tf::Transform &input, const ros::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
    br.sendTransform(tf::StampedTransform(T_WorldLink, time, "map", camera_slam_frame_id)); // camera_link frame w.r.t. map frame

#endif
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD)
{
    if(!bGotCameraInfo) 
    {
        std::cout << "lost frame for waiting camera info" << std::endl; 
        return; 
    }
    
    std::unique_lock<mutex> lck(grab_mutex);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //std::cout << "image size: " << cv_ptrRGB->image.size << ", channels " << cv_ptrRGB->image.channels() << std::endl; 
     
    
    cv::Mat convertedImg = cv_ptrRGB->image;  
    if ( cv_ptrRGB->image.channels() == 4 ) 
    {
        //std::cout << "converting image from 4 channels to 3 channels" << std::endl;         
        if (!bRGB)
        {
            cv::cvtColor(convertedImg, convertedImg, cv::COLOR_RGBA2BGR, 3);
        }  
        else
        {
            cv::cvtColor(convertedImg, convertedImg, cv::COLOR_RGBA2RGB, 3);                 
        }
    }   
    else 
    {
        if (!bRGB)
        {
            cv::cvtColor(convertedImg, convertedImg, cv::COLOR_RGB2BGR, 3);
        }        
    }
    

    // w = WORLD
    // o = OPTICAL FRAME
    // l = LINK FRAME
    
    Sophus::SE3f T_ow = mpSLAM->TrackRGBD(convertedImg, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    trackingState = mpSLAM->GetTrackingState();


    //if (T_ow.empty())
    if( trackingState == PLVS2::Tracking::LOST)
    {
        if (!b_was_lost) ROS_WARN_STREAM("mpSLAM->TrackRGBD() returned empty pose");
        b_was_lost = true;
        T_ow = T_wo_zero;
        //return;
    }
    else
    {
        if (b_was_lost) ROS_INFO_STREAM("mpSLAM->TrackRGBD() returned valid pose");
        b_was_lost = false;
    }

    //static cv::Mat pose_inv  = cv::Mat::eye(4,4, CV_32F);

    //pose_inv = pose.inv(); // T_wc
    T_wo = T_ow.inverse();

    // pose = pose_inv*T_co-cl; // T_wcl
    T_wl = T_wo*T_ol;

    //std::cout << "tracking state: " << trackingState << std::endl;

    if(!bUseViewer)
        SendTransform(cv_ptrRGB->header.stamp);

}
