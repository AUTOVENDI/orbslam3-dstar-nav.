#include "rgbd-slam-node_UPDATED.hpp"
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <set>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <cmath>

using std::placeholders::_1;

RGBDSLAMNode::RGBDSLAMNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_RGBD"), slam_system_(pSLAM)
{
    rgb_subscriber_ = this->create_subscription<ImageMsg>(
        "/camera/image_raw", 10,
        std::bind(&RGBDSLAMNode::GrabRGB, this, _1));

    depth_subscriber_ = this->create_subscription<ImageMsg>(
        "/camera/depth/image_raw", 10,
        std::bind(&RGBDSLAMNode::GrabDepth, this, _1));

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "orbslam3/map", 10);

    // --- [ NEW ] ---
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "orbslam3/pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::cout << "RGBD SLAM Node fully initialized." << std::endl;
}

RGBDSLAMNode::~RGBDSLAMNode()
{
    slam_system_->Shutdown();
    slam_system_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ORB_SLAM3::Atlas* atlas = slam_system_->GetAtlas();
    if (!atlas) {
        RCLCPP_ERROR(this->get_logger(), "Failed to access Atlas from System.");
        return;
    }

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = atlas->GetAllKeyFrames();
    std::set<ORB_SLAM3::MapPoint*> all_map_points;

    for (auto kf : keyframes) {
        if (kf) {
            auto mps_set = kf->GetMapPoints();
            for (auto mp : mps_set) {
                if (mp && !mp->isBad()) {
                    all_map_points.insert(mp);
                }
            }
        }
    }

    std::vector<ORB_SLAM3::MapPoint*> map_points(all_map_points.begin(), all_map_points.end());
    SaveMapAsPLY(map_points, "orbslam3_rgbd_map.ply");
}

void RGBDSLAMNode::SaveMapAsPLY(const std::vector<ORB_SLAM3::MapPoint*>& map_points, const std::string& filename)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto& mp : map_points) {
        if (mp && !mp->isBad()) {
            auto pos = mp->GetWorldPos();
            cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }
    pcl::io::savePLYFileASCII(filename, cloud);
    RCLCPP_INFO(this->get_logger(), "Saved map as PLY: %s", filename.c_str());
}

void RGBDSLAMNode::GrabRGB(const ImageMsg::SharedPtr msg)
{
    try {
        cv_rgb_ = cv_bridge::toCvCopy(msg);
        ProcessRGBD();
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (RGB): %s", e.what());
    }
}

void RGBDSLAMNode::GrabDepth(const ImageMsg::SharedPtr msg)
{
    try {
        cv_depth_ = cv_bridge::toCvCopy(msg);
        ProcessRGBD();
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (Depth): %s", e.what());
    }
}

void RGBDSLAMNode::ProcessRGBD()
{
    if (!cv_rgb_ || !cv_depth_) return;

    // Process RGBD images through SLAM
    Sophus::SE3f camera_pose = slam_system_->TrackRGBD(cv_rgb_->image, cv_depth_->image, this->now().seconds());

    if (camera_pose.translation().norm() > 1e-6)  // check if non-trivial pose
    {
        PublishPose(camera_pose);  // --- [ NEW ] Publish pose and TF
    }

    PublishPointCloud();
}

void RGBDSLAMNode::PublishPointCloud()
{
    ORB_SLAM3::Atlas* atlas = slam_system_->GetAtlas();
    if (!atlas) return;

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = atlas->GetAllKeyFrames();
    std::set<ORB_SLAM3::MapPoint*> all_map_points;

    for (auto kf : keyframes) {
        if (kf) {
            auto mps_set = kf->GetMapPoints();
            all_map_points.insert(mps_set.begin(), mps_set.end());
        }
    }

    std::vector<ORB_SLAM3::MapPoint*> map_points(all_map_points.begin(), all_map_points.end());

    auto cloud_msg = ConvertToPointCloud2(map_points);
    pointcloud_publisher_->publish(cloud_msg);
}

sensor_msgs::msg::PointCloud2 RGBDSLAMNode::ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto& mp : map_points) {
        if (mp && !mp->isBad()) {
            auto pos = mp->GetWorldPos();
            cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "odom";
    msg.header.stamp = this->now();
    return msg;
}

// --- [ NEW FUNCTION ] ---
void RGBDSLAMNode::PublishPose(const Sophus::SE3f& pose)
{
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = pose.translation().x();
    pose_msg.pose.position.y = pose.translation().y();
    pose_msg.pose.position.z = pose.translation().z();

    Eigen::Quaternionf q(pose.unit_quaternion());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_publisher_->publish(pose_msg);

    // Publish TF from map -> odom
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = pose.translation().x();
    transformStamped.transform.translation.y = pose.translation().y();
    transformStamped.transform.translation.z = pose.translation().z();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tfBroadcaster_->sendTransform(transformStamped);
}

