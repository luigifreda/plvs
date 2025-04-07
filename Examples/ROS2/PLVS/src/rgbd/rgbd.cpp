#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bool bRGB = static_cast<bool> ((int) fSettings["Camera.RGB"]);
    bool bUseViewer = static_cast<int> (PLVS2::Utils::GetParam(fSettings, "Viewer.on", 1)) != 0;
    bool bWaitForCameraInfo = static_cast<int> (PLVS2::Utils::GetParam(fSettings, "Camera.waitCameraInfoOn", 0)) != 0;
    
    float baseline = static_cast<float>(fSettings["Camera.bf"])/static_cast<float>(fSettings["Camera.fx"]);

    if(!bUseViewer)
    {
        // If the user wants to disable the viewer, we invert the RGB fields for RVIZ visualization.
        std::cout << "Pangolin viewer is disabled => inverting RGB fields for RVIZ visualization" << std::endl;
        bRGB = !bRGB;
    }

    auto pSLAM = std::make_shared<PLVS2::System>(argv[1], argv[2], PLVS2::System::RGBD, bUseViewer);
    auto node = std::make_shared<RgbdSlamNode>(pSLAM, bWaitForCameraInfo);
    node->SetBaseline(baseline);    
    std::cout << "============================ " << std::endl;

#if  1
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // Use 4 threads
    executor.add_node(node);  
    executor.spin();          // block until shutdown
    rclcpp::shutdown();
#else
    rclcpp::spin(node);
    rclcpp::shutdown();
#endif 

    return 0;
}
