#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

class RgbdSlamNode : public rclcpp::Node
{
    static constexpr int kTimeOutWaitCameraInfoMs = 10000; // time out for waiting camera info [ms]
public:
    RgbdSlamNode(std::shared_ptr<PLVS2::System>& pSLAM, bool bWaitForCameraInfo = false);

    ~RgbdSlamNode();

public: 

    void SetBaseline(float baseline) { baseline_ = baseline; }

private: 

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);
    void GrabCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void WaitCameraInfoTimerCallback();    

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    std::shared_ptr<PLVS2::System> pSLAM_;

    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_color_sub;
    rclcpp::TimerBase::SharedPtr timerWaitCameraInfo;
    
    bool bWaitForCameraInfo_ = false;
    std::atomic_bool bGotCameraInfo_ = false;    
    std::mutex mMutexGotCameraInfo_;
    float baseline_ = 1.0f;
};

#endif
