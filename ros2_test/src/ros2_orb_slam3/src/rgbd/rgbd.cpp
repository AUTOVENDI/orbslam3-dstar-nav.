#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node_UPDATED.hpp"  // Your RGBDSLAMNode class
#include "System.h"                     // ORB_SLAM3 system

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "\nUsage: ros2 run ros2_orb_slam3 rgbd_node <path_to_vocabulary> <path_to_settings>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    std::string vocab_file = argv[1];
    std::string settings_file = argv[2];
    bool use_viewer = true;

    ORB_SLAM3::System SLAM(vocab_file, settings_file, ORB_SLAM3::System::RGBD, use_viewer);

    auto node = std::make_shared<RGBDSLAMNode>(&SLAM);
    std::cout << "✅ RGBD-SLAM node started successfully..." << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

