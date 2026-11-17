#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>

// PCL Libraries for Point Cloud
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

// ORB-SLAM3 Core Libraries
#include "System.h"
#include "Atlas.h"
#include "KeyFrame.h"
#include "MapPoint.h"

// Other Libraries
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <thread>
#include <atomic>

class MonocularSlamNode : public rclcpp::Node
{
public:
    // Constructor and Destructor
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();
    
private:
    using ImageMsg = sensor_msgs::msg::Image;

    // Image Callback Function
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    // Function to Publish the 3D Map as PointCloud2
    void PublishPointCloud();

    // Function to Convert ORB-SLAM3 MapPoints to PointCloud2
    sensor_msgs::msg::PointCloud2 ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points);

    // Occupancy Grid Functions
    void OccupancyGridLoop();
    void Bresenham(int r1, int c1, int r2, int c2);
    void DrawOccupancyMap(cv::Mat& canvas);
    bool check_boundary(int r, int c);

    // Save the map as PLY file (Optional if you still use it)
    void SaveMapAsPLY(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const std::string& filename);
    void PublishPose(const Sophus::SE3f& pose); // Optional TF publishing (comment if unused)

    // ORB-SLAM3 System Pointer
    ORB_SLAM3::System* m_SLAM;

    // CV Bridge for Image Conversion
    cv_bridge::CvImagePtr m_cvImPtr;

    // ROS2 Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_publisher;

    // Occupancy grid data
    static constexpr int cell_size = 800;
    static constexpr float res = 0.01f;
    std::atomic<unsigned long long> atomic_cnts[2][cell_size][cell_size];

    // Thread for occupancy grid updating
    std::thread occupancy_thread_;

    // Current pose of the camera
    Sophus::SE3f now_;

    // Run control flag
    bool running = true;
};

#endif // __MONOCULAR_SLAM_NODE_HPP__

