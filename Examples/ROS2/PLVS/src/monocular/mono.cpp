#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"

#include "System.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    if(args.size() < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool visualization = true;
    const std::string& vocabulary_file = args[1];
    const std::string& settings_file = args[2];
    PLVS2::System SLAM(vocabulary_file, settings_file, PLVS2::System::MONOCULAR, visualization);

    auto node = std::make_shared<MonocularSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;\

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
