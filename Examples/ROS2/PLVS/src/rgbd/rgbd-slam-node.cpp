#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Logger.h>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(std::shared_ptr<PLVS2::System>& pSLAM, bool bWaitForCameraInfo)
:   Node("PLVS2"),
    pSLAM_(pSLAM),
    bWaitForCameraInfo_(bWaitForCameraInfo)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");   // /camera/rgb/image_raw
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth"); // camera/depth_registered/image_raw

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    if(bWaitForCameraInfo_)
    {
        info_color_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera/rgb/camera_info", 10, std::bind(&RgbdSlamNode::GrabCameraInfo, this, std::placeholders::_1));
        // One-shot timer to handle timeout
        timerWaitCameraInfo = this->create_wall_timer(std::chrono::milliseconds(kTimeOutWaitCameraInfoMs),
            std::bind(&RgbdSlamNode::WaitCameraInfoTimerCallback, this));    
    } else {
        bGotCameraInfo_ = true;
    }

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    pSLAM_->Shutdown();

    // Save camera trajectory
    pSLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    if(!bGotCameraInfo_) 
    {
        RCLCPP_WARN(this->get_logger(), "lost frame for waiting camera info"); 
        return; 
    }

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    pSLAM_->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
}

void RgbdSlamNode::GrabCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    unique_lock<mutex> lock(mMutexGotCameraInfo_);
    if(!bGotCameraInfo_)
    {        
        std::cout << "camera info has been received and set" << std::endl;
        
        // 3x3 row-major matrix
        //     [fx  0 cx]
        // K = [ 0 fy cy]
        //     [ 0  0  1]        
        float fx = msg->k[0]; float cx = msg->k[2];
        float fy = msg->k[4]; float cy = msg->k[5];
        float bf = baseline_ * fx;

        // For "plumb_bob", D = [k1, k2, t1, t2, k3].        
        cv::Mat distCoef(5,1,CV_32F);
        distCoef.at<float>(0) = msg->d[0];
        distCoef.at<float>(1) = msg->d[1];
        distCoef.at<float>(2) = msg->d[2];
        distCoef.at<float>(3) = msg->d[3];
        distCoef.at<float>(4) = msg->d[4];   
        
        float imageScale = pSLAM_->GetImageScale();
        if(imageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx *= imageScale;
            fy *= imageScale;
            cx *= imageScale;
            cy *= imageScale;
            bf *= imageScale;
        }

        pSLAM_->SetCameraCalibration(fx, fy, cx, cy, distCoef, bf);
    
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
        
        bGotCameraInfo_ = true;         
    }
}

void RgbdSlamNode::WaitCameraInfoTimerCallback()
{
    unique_lock<mutex> lock(mMutexGotCameraInfo_);    
    if(!bGotCameraInfo_)
    {
        std::cout << "camera info was not received" << std::endl; 
        Logger logger("receivedCalibration.txt");
        logger << "none - used yml file";             
        bGotCameraInfo_ = true;
    }
    timerWaitCameraInfo->cancel(); // stop timer    
}
