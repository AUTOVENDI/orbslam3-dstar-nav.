#ifndef __RGBD_SLAM_NODE_HPP__
#define __RGBD_SLAM_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"  // Include PointCloud2 message type

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class RgbdSlamNode : public rclcpp::Node
{
public:
    RgbdSlamNode(ORB_SLAM3::System* pSLAM);

    ~RgbdSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    // Callback to process RGB-D data
    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

    // Function to publish PointCloud2 data (for RViz visualization)
    void PublishPointCloud();

    // Function to save the 3D map as a PLY file
    void SaveMapAsPLY();

    // Function to convert map points to PointCloud2 format
    sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points);  // Declare here

    // ORB-SLAM3 system pointer
    ORB_SLAM3::System* m_SLAM;

    // CvBridge pointers for RGB and Depth
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    cv_bridge::CvImageConstPtr cv_ptrD;

    // ROS 2 subscribers for RGB and Depth
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image> > depth_sub;

    // Synchronizer for approximate time sync between RGB and Depth
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy> > syncApproximate;

    // Publisher for PointCloud2 (to visualize 3D map in RViz)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;
};

#endif

