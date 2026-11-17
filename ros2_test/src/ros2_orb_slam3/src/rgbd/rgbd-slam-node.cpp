#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    // Subscribe to RGB and Depth topics
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/image_raw");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    // PointCloud2 publisher for RViz visualization
    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("orbslam3/map", 10);
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Optionally, save the map as a .ply file
    SaveMapAsPLY();
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
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

    // Process RGB-D data with ORB-SLAM3
    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    // Publish the real-time 3D map (PointCloud2) for visualization in RViz
    PublishPointCloud();
}

void RgbdSlamNode::PublishPointCloud()
{
    // Get the map from ORB-SLAM3's Atlas
    ORB_SLAM3::Atlas* atlas = m_SLAM->GetAtlas();
    if (!atlas) return;

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = atlas->GetAllKeyFrames();
    std::set<ORB_SLAM3::MapPoint*> all_map_points;

    for (auto kf : keyframes) {
        if (kf) {
            std::set<ORB_SLAM3::MapPoint*> mps_set = kf->GetMapPoints();
            all_map_points.insert(mps_set.begin(), mps_set.end());
        }
    }

    // Convert map points to PointCloud2 message for ROS
    std::vector<ORB_SLAM3::MapPoint*> map_points(all_map_points.begin(), all_map_points.end());
    sensor_msgs::msg::PointCloud2 pointcloud_msg = ConvertToPointCloud2(map_points);

    // Publish the PointCloud2 message
    m_pointcloud_publisher->publish(pointcloud_msg);
}

sensor_msgs::msg::PointCloud2 RgbdSlamNode::ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    // Create a PCL PointCloud and convert MapPoints into PointXYZ
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (const auto& mp : map_points) {
        if (mp && !mp->isBad()) {
            Eigen::Vector3f pos = mp->GetWorldPos();
            cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }

    // Convert PCL PointCloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);

    // Set the header for the PointCloud2 message
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    return msg;
}

void RgbdSlamNode::SaveMapAsPLY()
{
    // Save the 3D map as a .ply file
    ORB_SLAM3::Atlas* atlas = m_SLAM->GetAtlas();
    if (!atlas) return;

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = atlas->GetAllKeyFrames();
    std::set<ORB_SLAM3::MapPoint*> all_map_points;

    for (auto kf : keyframes) {
        if (kf) {
            std::set<ORB_SLAM3::MapPoint*> mps_set = kf->GetMapPoints();
            all_map_points.insert(mps_set.begin(), mps_set.end());
        }
    }

    std::vector<ORB_SLAM3::MapPoint*> map_points(all_map_points.begin(), all_map_points.end());

    // Save map points to a .ply file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& map_point : map_points) {
        if (map_point && !map_point->isBad()) {
            Eigen::Vector3f pos = map_point->GetWorldPos();
            cloud->points.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    // Save as PLY file
    pcl::io::savePLYFileBinary("orbslam3_rgbd_map.ply", *cloud);
    RCLCPP_INFO(rclcpp::get_logger("ORB-SLAM3"), "Saved 3D map as orbslam3_rgbd_map.ply");
}


