#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    if(args.size() < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    const std::string& vocabulary_file = args[1];
    const std::string& settings_file = args[2];

    cv::FileStorage fSettings(settings_file, cv::FileStorage::READ);
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

    auto pSLAM = std::make_shared<PLVS2::System>(vocabulary_file, settings_file, PLVS2::System::RGBD, bUseViewer);
    auto node = std::make_shared<RgbdSlamNode>(pSLAM, bWaitForCameraInfo);
    node->SetBaseline(baseline);
    // Initialize subscribers after node is fully constructed (required for shared_from_this())
    node->InitializeSubscribers();
    std::cout << "============================ " << std::endl;

#if  1
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4); // Use 4 threads
    executor.add_node(node);  
    std::cout << "Starting executor.spin() - waiting for ROS2 messages..." << std::endl;
    executor.spin();          // block until shutdown (Ctrl+C or rclcpp::shutdown())
    std::cout << "executor.spin() returned" << std::endl;
    executor.remove_node(node);
#else
    std::cout << "Starting rclcpp::spin() - waiting for ROS2 messages..." << std::endl;
    rclcpp::spin(node);
    std::cout << "rclcpp::spin() returned" << std::endl;
#endif 

    std::cout << "Calling rclcpp::shutdown()" << std::endl;
    rclcpp::shutdown();

    return 0;
}
