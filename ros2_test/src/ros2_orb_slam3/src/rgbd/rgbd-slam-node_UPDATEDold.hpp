#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cv_bridge/cv_bridge.h>
#include "System.h"  // ORB_SLAM3 System header
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>

class RGBDSLAMNode : public rclcpp::Node {
public:
    using ImageMsg = sensor_msgs::msg::Image;

    explicit RGBDSLAMNode(ORB_SLAM3::System* pSLAM);
    ~RGBDSLAMNode();

private:
    void GrabRGB(const ImageMsg::SharedPtr msg);
    void GrabDepth(const ImageMsg::SharedPtr msg);
    void ProcessRGBD();
    void PublishPointCloud();
    void SaveMapAsPLY(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const std::string& filename);
    sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    void PublishPose(const Sophus::SE3f& pose);  // Publish PoseStamped + map -> odom TF

    // TF and ROS publishers
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    rclcpp::Subscription<ImageMsg>::SharedPtr rgb_subscriber_;
    rclcpp::Subscription<ImageMsg>::SharedPtr depth_subscriber_;

    ORB_SLAM3::System* slam_system_;

    cv_bridge::CvImagePtr cv_rgb_;
    cv_bridge::CvImagePtr cv_depth_;
};

