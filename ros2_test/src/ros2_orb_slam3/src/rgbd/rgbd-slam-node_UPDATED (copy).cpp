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
#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

using std::placeholders::_1;

const int cell_size = 1000;
atomic<unsigned long long> atomic_cnts[2][cell_size][cell_size]; // 0:visited, 1:occupied
bool flag = 1;
Sophus::SE3f current_pose;

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

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "orbslam3/pose", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);

    std::cout << "RGBD SLAM Node fully initialized." << std::endl;
    std::thread(&RGBDSLAMNode::occupancy_grid, this, std::ref(*slam_system_)).detach();


}

RGBDSLAMNode::~RGBDSLAMNode()
{
    flag = false; // stop the occupancy thread
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    saveOccupancyGridAsCSV("occupancy_grid.csv"); // now it's safe to save
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
    //saveOccupancyGridAsCSV("occupancy_grid.csv");
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

    // Ensure both images are available
    if (!cv_rgb_ || !cv_depth_)
        return;

    // Optionally add a time sync threshold check here if you need better alignment

    Sophus::SE3f camera_pose = slam_system_->TrackRGBD(
        cv_rgb_->image,
        cv_depth_->image,
        this->now().seconds()  // Or use rgb_timestamp_.seconds() if needed
    );
    current_pose = camera_pose;  // ✅ Add this
    // Use the updated pose to publish and update
    if (camera_pose.translation().norm() > 1e-6)
    {
        PublishPose(camera_pose);
        UpdateOccupancyGrid();  // ✅ Keep this as requested
    }

    PublishPointCloud();
}
void RGBDSLAMNode::UpdateOccupancyGrid()
{
    auto keyframes = slam_system_->GetAtlas()->GetAtlasKeyframes();
    if (keyframes.empty()) return;

    ORB_SLAM3::KeyFrame* latest_kf = keyframes.rbegin()->second;  // Gets the last (most recent) keyframe

    const Sophus::SE3f pose = latest_kf->GetPose();
    const auto map_points = latest_kf->GetMapPoints();
    const float resolution = 0.06;
    for (auto& mp : map_points)
    {
        const auto world_position = mp->GetWorldPos();
        int x_cell = cell_size / 2 + static_cast<int>(world_position(0) / resolution);
        int y_cell = cell_size / 2 - static_cast<int>(world_position(2) / resolution);

        if (x_cell >= 0 && x_cell < cell_size && y_cell >= 0 && y_cell < cell_size)
        {
            atomic_cnts[0][x_cell][y_cell].fetch_add(1);
            if (world_position(1) < 0)
                atomic_cnts[1][x_cell][y_cell].fetch_add(1);
        }
    }

    cv::Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120));
    drawOccupancyMap(canvas);
    cv::imshow("Occupancy Grid", canvas);
    cv::waitKey(1);
}

bool RGBDSLAMNode::check_boundary(int r, int c)
{
    return r >= 0 && c >= 0 && r < cell_size && c < cell_size;
}

void RGBDSLAMNode::bresenham(int r1, int c1, int r2, int c2)
{
    if (check_boundary(r2, c2))
        atomic_cnts[1][r2][c2].fetch_add(1);

    if (c1 == c2)
    {
        if (r1 > r2)
            std::swap(r1, r2);

        while (r1 <= r2)
        {
            if (!check_boundary(r1, c1))
                break;
            atomic_cnts[0][r1][c1].fetch_add(1);
            r1++;
        }
    }
    else
    {
        if (c1 > c2)
        {
            swap(c1, c2);
            swap(r1, r2);
        }
        if (r1 == r2)
        {
            while (c1 <= c2)
            {
                if (!check_boundary(r1, c1))
                    break;
                atomic_cnts[0][r1][c1].fetch_add(1);
                c1++;
            }
        }
        else
        {
            if (r1 > r2)
            {
                r2 = r1 + (r1 - r2);
                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dr <= dc)
                {
                    const int r0 = r1;
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(r0 - (r1 - r0), c1))
                            break;
                        atomic_cnts[0][r0 - (r1 - r0)][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    const int c0 = c1;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(c0 - (c1 - c0), r1))
                            break;
                        atomic_cnts[0][c0 - (c1 - c0)][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
            else
            {
                int dr = r2 - r1;
                int dc = c2 - c1;

                if (dc >= dr)
                {
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(r1, c1))
                            break;
                        atomic_cnts[0][r1][c1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
                else
                {
                    swap(dr, dc);
                    swap(c1, r1);
                    swap(c2, r2);
                    int p = 2 * dr - dc;
                    while (c1 <= c2)
                    {
                        if (!check_boundary(c1, r1))
                            break;
                        atomic_cnts[0][c1][r1].fetch_add(1);
                        c1++;
                        if (p < 0)
                            p = p + 2 * dr;
                        else
                        {
                            p = p + 2 * dr - 2 * dc;
                            r1++;
                        }
                    }
                }
            }
        }
    }
}

void RGBDSLAMNode::drawOccupancyMap(cv::Mat &canvas)
{
    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            int visit_cnt = 0;
            int occupy_cnt = 0;
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    if (!check_boundary(i + dr, j + dc))
                        continue;
                    visit_cnt += atomic_cnts[0][i + dr][j + dc];
                    occupy_cnt += atomic_cnts[1][i + dr][j + dc];
                }
            }
            if (visit_cnt < 5)
                continue;

            int percent = (occupy_cnt * 100) / visit_cnt;
            if (percent >= 15)
            {
                cv::circle(canvas, cv::Point(j, i), 0, cv::Scalar(0, 0, 0), 3);
            }
            else
            {
                cv::circle(canvas, cv::Point(j, i), 0, cv::Scalar(255, 255, 255));
            }
        }
    }
}

void RGBDSLAMNode::occupancy_grid(ORB_SLAM3::System &SLAM)
{
    cv::Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120)); 
    const float res = 0.01;

    while (flag)
    {
        #pragma omp parallel for schedule(dynamic, 1) collapse(2)
        for (int i = 0; i < cell_size; ++i)
        {
            for (int j = 0; j < cell_size; ++j)
            {
                atomic_cnts[0][i][j].store(0);
                atomic_cnts[1][i][j].store(0);
            }
        }

        canvas.setTo(cv::Scalar(120, 120, 120));
        const auto mps = SLAM.GetAtlas()->GetAllMapPoints();
        const int mps_len = mps.size();

        #pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < mps_len; ++i)
        {
            const auto p = mps[i]->GetWorldPos();
            const int c = cell_size / 2 + (int)(p(0) / res);
            const int r = cell_size / 2 - (int)(p(2) / res);
            const int z = (int)(p(1) / res);
            const auto p0 = mps[i]->GetReferenceKeyFrame()->GetPose().inverse().translation();
            const int c0 = cell_size / 2 + (int)(p0(0) / res);
            const int r0 = cell_size / 2 - (int)(p0(2) / res);
            const int z0 = (int)(p0(1) / res);
            if(abs(z - z0) > (int)(1.f / res)) continue;
            bresenham(r0, c0, r, c);
        }

        drawOccupancyMap(canvas);

        const Sophus::Vector3f trans = current_pose.translation();
        const Sophus::Vector3f dir = current_pose.rotationMatrix().col(2);
        const int c = cell_size / 2 + (int)(trans(0) / res); 
        const int r = cell_size / 2 + (int)(trans(2) / res);
        const int ratio[2] = {-(int)(dir(0) / res), (int)(dir(2) / res)};
        const int dc = (20*ratio[0]) / (abs(ratio[0]) + abs(ratio[1]));
        const int dr = (20*ratio[1]) / (abs(ratio[0]) + abs(ratio[1]));
        if (check_boundary(r, c))
        {
            cv::circle(canvas, cv::Point(c, r), 0, cv::Scalar(0, 0, 255), 10);
            cv::arrowedLine(canvas, cv::Point(c,r), cv::Point(c + dc, r + dr), cv::Scalar(0,0,255),2, cv::LINE_8, 0, 0.5);
        }

        cv::imshow("Canvas", canvas);
        cv::waitKey(1);
        usleep(100 * 1000);
        int key = cv::waitKey(1);

    }
}
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
void RGBDSLAMNode::saveOccupancyGridAsCSV(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            int visit_cnt = atomic_cnts[0][i][j].load();
            int occupy_cnt = atomic_cnts[1][i][j].load();

            float occupancy = 0.0f;
            if (visit_cnt > 0) {
                occupancy = static_cast<float>(occupy_cnt) / visit_cnt;
            }

            file << occupancy;
            if (j < cell_size - 1)
                file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Occupancy grid saved to: " << filename << std::endl;
}

