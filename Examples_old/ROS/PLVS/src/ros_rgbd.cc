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
#include <tf2_ros/static_transform_broadcaster.h>

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


using namespace std;

bool bRGB = true;
bool bUseViewer = true;

std::shared_ptr<PLVS2::System> pSLAM;

class ImageGrabber;
std::shared_ptr<ImageGrabber> igb;

std::shared_ptr<PLVS2::PointCloudMapping> pPointCloudMapping;
ros::Publisher pointCloudPublisher;
std::uint64_t map_timestamp = 0;

int trackingState = (int)PLVS2::Tracking::NOT_INITIALIZED;

std::string map_frame_id = "map";
std::string camera_zero_frame_id = "camera_optical_zero";
std::string camera_optical_frame_id = "camera_optical_slam";
std::string camera_link_frame_id = "camera_link_slam";

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
        // T_ol = (cv::Mat_<float>(4, 4) << 0, -1, 0, 0,
        //         0, 0, -1, 0,
        //         1, 0, 0, 0,
        //         0, 0, 0, 1);

        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << 0, -1, 0,  0, 0, -1,  1, 0, 0;
        Eigen::Vector3f translation;
        translation << 0, 0, 0;
        T_ol = Sophus::SE3f(rotation_matrix, translation);

        T_mw = T_ol.inverse();
    }

    void SetSlamSystem(PLVS2::System* pSLAM)
    {
        mpSLAM = pSLAM;

        imageScale = pSLAM->GetImageScale();
    }

    void SendInitialMapTransform();
    void SendTransform(const ros::Time& time);

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);

    void SetMaxDisparity(float value) { maxDisparity = value; }

protected:

    PLVS2::System* mpSLAM;
    bool b_was_lost;
    bool b_is_first_frame = true;

    tf::TransformBroadcaster br;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_mw;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_lo;    

    // m = MAP 
    // w = WORLD  (first optical frame for slam)
    // o = OPTICAL FRAME
    // l = LINK FRAME
    Sophus::SE3f T_wo;
    Sophus::SE3f T_wl;
    Sophus::SE3f T_ol;
    Sophus::SE3f T_mw;

    std::mutex grab_mutex;

    float imageScale = 1.0f; 
    float maxDisparity = 1.0f; 
    size_t frameCounter = 0; 
};


#define USE_POINTCLOUD_MAPPING 1
#if USE_POINTCLOUD_MAPPING

template<typename PointT>   
void convertToRosMsg(const pcl::PointCloud<PointT>& cloud, sensor_msgs::PointCloud2& out_pcl)
{
    // see https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743#id_token=eyJhbGciOiJSUzI1NiIsImtpZCI6ImY4MzNlOGE3ZmUzZmU0Yjg3ODk0ODIxOWExNjg0YWZhMzczY2E4NmYiLCJ0eXAiOiJKV1QifQ.eyJpc3MiOiJodHRwczovL2FjY291bnRzLmdvb2dsZS5jb20iLCJhenAiOiIyMTYyOTYwMzU4MzQtazFrNnFlMDYwczJ0cDJhMmphbTRsamRjbXMwMHN0dGcuYXBwcy5nb29nbGV1c2VyY29udGVudC5jb20iLCJhdWQiOiIyMTYyOTYwMzU4MzQtazFrNnFlMDYwczJ0cDJhMmphbTRsamRjbXMwMHN0dGcuYXBwcy5nb29nbGV1c2VyY29udGVudC5jb20iLCJzdWIiOiIxMTY5ODM3MzIyNjg1Nzk2ODcyNjQiLCJlbWFpbCI6Imx1aWdpZnJlZGFAZ21haWwuY29tIiwiZW1haWxfdmVyaWZpZWQiOnRydWUsIm5iZiI6MTY5OTg3OTI1OSwibmFtZSI6Ikx1aWdpIEZyZWRhIiwicGljdHVyZSI6Imh0dHBzOi8vbGgzLmdvb2dsZXVzZXJjb250ZW50LmNvbS9hL0FDZzhvY0tEcE5JUXVGUXJoWDVHLXhIV3EyZFc0TElxRGRncGhzT2F1SEFyYlJFSmxYMD1zOTYtYyIsImdpdmVuX25hbWUiOiJMdWlnaSIsImZhbWlseV9uYW1lIjoiRnJlZGEiLCJsb2NhbGUiOiJpdCIsImlhdCI6MTY5OTg3OTU1OSwiZXhwIjoxNjk5ODgzMTU5LCJqdGkiOiI1NTFkYWYyNjg5MGM0MjYxMWQzMWM4Mjg5ZGRjM2EwMTY5ZmY2Y2Q4In0.XqEMEFVZysjB2bPPqHabJUtySu4DBzK0iXem7xGgJ0sTiBvzNX8X_21cVld3MRh7eR7RJiSduc7iSw5QCTGtEFpy57q__wBpQJ4W7o-nFlU_yZzIp7_9Q_pBegA_-zPA26ewLbzuICcnW-fh1OXhZGInsrA-0CA4CLnpmAvOHJ5l4uxWxzyJGaVGyCjct1EXlkk-2eccu0DeiQc2xZNLdjlT-8FfDcoScjm0Y9SZDm8-SoE2f8YaPToG1bOhOFhWW2he_hVHJ6bo2Kc--QcbRU5znkumo_OdCFPueGW-74O3tZz-47qqret4eyVS28YDl4Gaoc5ZoybBTsr-9A_5dw
    out_pcl.header.frame_id = cloud.header.frame_id;
    
    ros::Time stamp; 
    stamp.fromNSec(cloud.header.stamp * 1000ull);
    out_pcl.header.stamp = stamp; 

    out_pcl.header.seq = cloud.header.seq;
    out_pcl.width = 0;
    out_pcl.height = 1;  

    out_pcl.fields.resize(4);
    out_pcl.fields[0].name = "x";
    out_pcl.fields[0].offset = 0;
    out_pcl.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    out_pcl.fields[0].count = 1;

    out_pcl.fields[1].name = "y";
    out_pcl.fields[1].offset = 4; // out_pcl.fields[0].offset + "sizeof(sensor_msgs::PointField::FLOAT32)""
    out_pcl.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    out_pcl.fields[1].count = 1;

    out_pcl.fields[2].name = "z";
    out_pcl.fields[2].offset = 8; // out_pcl.fields[1].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"
    out_pcl.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    out_pcl.fields[2].count = 1;

    out_pcl.fields[3].name = "rgb";
    out_pcl.fields[3].offset = 12; // out_pcl.fields[2].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"
    out_pcl.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    out_pcl.fields[3].count = 1;
   

    const uint32_t POINT_STEP = 16; // out_pcl.fields[3].offset + "sizeof(sensor_msgs::PointField::FLOAT32)"

    out_pcl.point_step = POINT_STEP;
    out_pcl.is_dense = true;  // dense means that there are no invalid points
    out_pcl.is_bigendian = false;

    out_pcl.data.clear();
    out_pcl.data.reserve(POINT_STEP*cloud.size());

    struct PointRGBA
    {
        float x, y, z, rgb; 
    } pointRGBA;

    // traverse point cloud
    for (size_t i = 0; i < cloud.size(); i++)
    {
        pointRGBA.x = cloud[i].x;
        pointRGBA.y = cloud[i].y;
        pointRGBA.z = cloud[i].z;
        pointRGBA.rgb = cloud[i].rgb;

        const char* ptr = reinterpret_cast<const char*> (&pointRGBA);
        out_pcl.width++;
        out_pcl.data.insert(out_pcl.data.end(), ptr, ptr + sizeof(pointRGBA));
    }
    out_pcl.row_step = out_pcl.data.size() * POINT_STEP;  // we set out_pcl.height = 1  
}

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
            pMap->header.frame_id = camera_zero_frame_id;
            sensor_msgs::PointCloud2 map_msg;
            //pcl::toROSMsg(*pMap, map_msg);
            convertToRosMsg<PLVS2::PointCloudMapping::PointT>(*pMap, map_msg);
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
        float fx = msg->K[0]; float cx = msg->K[2];
        float fy = msg->K[4]; float cy = msg->K[5];
        float bf = baseline * fx;

        // For "plumb_bob", D = [k1, k2, t1, t2, k3].        
        cv::Mat distCoef(5,1,CV_32F);
        distCoef.at<float>(0) = msg->D[0];
        distCoef.at<float>(1) = msg->D[1];
        distCoef.at<float>(2) = msg->D[2];
        distCoef.at<float>(3) = msg->D[3];
        distCoef.at<float>(4) = msg->D[4];   
        
        float imageScale = pSLAM->GetImageScale();
        if(imageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx *= imageScale;
            fy *= imageScale;
            cx *= imageScale;
            cy *= imageScale;
            bf *= imageScale;
        }

        igb->SetMaxDisparity(fx);  

        pSLAM->SetCameraCalibration(fx, fy, cx, cy, distCoef, bf);
    
        std::stringstream ss;
        ss << std::fixed << std::setprecision(8);
        
        ss << "imageScale: " << imageScale << std::endl;        
        ss << "width: " << msg->width << std::endl; 
        ss << "height: " << msg->height << std::endl; 

        ss << "fx: " << fx << std::endl;
        ss << "fy: " << fy << std::endl;
        ss << "cx: " << cx << std::endl;
        ss << "cy: " << cy << std::endl;           
        ss << "bf: " << bf << std::endl;              
        ss << "distCoef: " << distCoef << std::endl;   

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

    if(!bUseViewer)
    {
        // If the user wants to disable the viewer, we invert the RGB fields for RVIZ visualization.
        ROS_WARN_STREAM("Pangolin viewer is disabled => inverting RGB fields for RVIZ visualization");
        bRGB = !bRGB;
    }


    igb = std::make_shared<ImageGrabber>();
    igb->SendInitialMapTransform();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    pSLAM = std::make_shared<PLVS2::System>(argv[1], argv[2], PLVS2::System::RGBD, bUseViewer);

    igb->SetSlamSystem(pSLAM.get());

    ros::NodeHandle nh;

    const int subscriber_queue_size=1; // real-time 1

    /// < TODO: use only rectified color images (otherwise we have to manage the undistortion inside the point cloud mapping)
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", subscriber_queue_size);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", subscriber_queue_size);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, igb, _1, _2));
    
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
        pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/plvs/pointcloud_map", 1, true);
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

    auto timestamp = ros::Time::now();

    SendTransform(timestamp);

    geometry_msgs::TransformStamped tf_map_world;
    tf_map_world.header.stamp = timestamp;
    tf_map_world.header.frame_id = map_frame_id;
    tf_map_world.child_frame_id = camera_zero_frame_id;
    auto translation_mw = T_mw.translation();
    auto rotation_mw = T_mw.unit_quaternion();
    tf_map_world.transform.translation.x = translation_mw.x();
    tf_map_world.transform.translation.y = translation_mw.y();
    tf_map_world.transform.translation.z = translation_mw.z();
    tf_map_world.transform.rotation.x = rotation_mw.x();
    tf_map_world.transform.rotation.y = rotation_mw.y();
    tf_map_world.transform.rotation.z = rotation_mw.z();
    tf_map_world.transform.rotation.w = rotation_mw.w();
    static_broadcaster_mw.sendTransform(tf_map_world);        

    geometry_msgs::TransformStamped tf_optical_link;
    tf_optical_link.header.stamp = timestamp;
    tf_optical_link.header.frame_id = camera_optical_frame_id;
    tf_optical_link.child_frame_id = camera_link_frame_id;
    auto translation_ol = T_ol.translation();
    auto rotation_ol = T_ol.unit_quaternion();
    tf_optical_link.transform.translation.x = translation_ol.x();
    tf_optical_link.transform.translation.y = translation_ol.y();
    tf_optical_link.transform.translation.z = translation_ol.z();
    tf_optical_link.transform.rotation.x = rotation_ol.x();
    tf_optical_link.transform.rotation.y = rotation_ol.y();
    tf_optical_link.transform.rotation.z = rotation_ol.z();
    tf_optical_link.transform.rotation.w = rotation_ol.w();
    static_broadcaster_lo.sendTransform(tf_optical_link);
}

void ImageGrabber::SendTransform(const ros::Time& time)
{
    Sophus::SE3f::Transformation T_wo_mat = T_wo.matrix(); 
    tf::Matrix3x3 cameraRotation(T_wo_mat(0, 0), T_wo_mat(0, 1), T_wo_mat(0, 2),
                                 T_wo_mat(1, 0), T_wo_mat(1, 1), T_wo_mat(1, 2),
                                 T_wo_mat(2, 0), T_wo_mat(2, 1), T_wo_mat(2, 2));                                 
    tf::Vector3 cameraTranslation(T_wo_mat(0, 3), T_wo_mat(1, 3), T_wo_mat(2, 3));

    tf::Transform T_WorldOptical = tf::Transform(cameraRotation, cameraTranslation);
    br.sendTransform(tf::StampedTransform(T_WorldOptical, time, camera_zero_frame_id, camera_optical_frame_id));
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
    cv::Mat depthImg = cv_ptrD->image;
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
    
    if(imageScale != 1.f)
    {
        int width = convertedImg.cols * imageScale;
        int height = convertedImg.rows * imageScale;
        cv::resize(convertedImg, convertedImg, cv::Size(width, height));
        cv::resize(depthImg, depthImg, cv::Size(width, height));        
    }

    if(0)
    {
        // DEBUG: save rgb and depth image and check their alignment 
        cv::Mat depthForBlend, coloredDepthImage;   
        double minVal, maxVal;
        cv::minMaxLoc(depthImg, &minVal, &maxVal);
        float scale = 5.0 * 255.0 / (maxVal - minVal);
        depthImg.convertTo(depthForBlend, CV_8UC1, scale);
        cv::applyColorMap(depthForBlend, coloredDepthImage, cv::COLORMAP_HOT);    

        cv::Mat blend;
        float rgbWeight = 0.4;
        float depthWeight = 0.6;
        cv::addWeighted(convertedImg, rgbWeight, coloredDepthImage, depthWeight, 0, blend);

        std::stringstream ssb;
        ssb << "/tmp/blend" << frameCounter << ".png";
        cv::imwrite(ssb.str(), blend);

        std::stringstream ssd;
        ssd << "/tmp/depth" << frameCounter << ".png";
        cv::imwrite(ssd.str(), coloredDepthImage);    

        frameCounter = (frameCounter+1) % 10; 
    }

    // w = WORLD
    // o = OPTICAL FRAME
    // l = LINK FRAME
    
    Sophus::SE3f T_ow = mpSLAM->TrackRGBD(convertedImg, depthImg, cv_ptrRGB->header.stamp.toSec());
    trackingState = mpSLAM->GetTrackingState();


    //if (T_ow.empty())
    if( trackingState == PLVS2::Tracking::LOST)
    {
        if (!b_was_lost) ROS_WARN_STREAM("mpSLAM->TrackRGBD() returned empty pose");
        b_was_lost = true;
        T_ow = Sophus::SE3f();
        //return;
    }
    else
    {
        if (b_was_lost) ROS_INFO_STREAM("mpSLAM->TrackRGBD() returned valid pose");
        b_was_lost = false;
    }

    T_wo = T_ow.inverse();
    T_wl = T_wo*T_ol;

    //std::cout << "tracking state: " << trackingState << std::endl;

    if(!bUseViewer)
        SendTransform(cv_ptrRGB->header.stamp);

}
