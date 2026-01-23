#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    if(args.size() < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    const std::string& vocabulary_file = args[1];
    const std::string& settings_file = args[2];
    const std::string& do_rectify = args[3];
    const std::string do_equalize = (args.size() >= 5) ? args[4] : "false";

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    PLVS2::System pSLAM(vocabulary_file, settings_file, PLVS2::System::IMU_STEREO, visualization);

    auto node = std::make_shared<StereoInertialNode>(&pSLAM, settings_file, do_rectify, do_equalize);
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
