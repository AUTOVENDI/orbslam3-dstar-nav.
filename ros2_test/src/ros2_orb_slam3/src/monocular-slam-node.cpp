#include "monocular-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <set>
#include <fstream>
#include <atomic>
#include <thread>
#include <omp.h>
#include <sophus/se3.hpp>

using std::placeholders::_1;

// Global Occupancy Map Variables
const int cell_size = 800;
const float res = 0.01f; // 1 cm per cell
std::atomic<unsigned long long> atomic_cnts[2][cell_size][cell_size];
bool running = true;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
: Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;

    // Initialize occupancy grid
    for (int i = 0; i < cell_size; ++i)
        for (int j = 0; j < cell_size; ++j)
            atomic_cnts[0][i][j] = atomic_cnts[1][i][j] = 0;

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, _1));

    m_pointcloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "orbslam3/map",
        10);

    occupancy_thread_ = std::thread(&MonocularSlamNode::OccupancyGridLoop, this);

    std::cout << "Monocular SLAM Node initialized." << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    running = false;
    if (occupancy_thread_.joinable())
        occupancy_thread_.join();

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Save the occupancy grid to a CSV file
    std::ofstream csv_file("occupancy_grid.csv");
    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            csv_file << (atomic_cnts[1][i][j] > 0 ? 1 : 0); // Save as binary (0 or 1)
            if (j < cell_size - 1)
                csv_file << ",";
        }
        csv_file << "\n";
    }
    csv_file.close();
    RCLCPP_INFO(this->get_logger(), "Saved occupancy grid to occupancy_grid.csv");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
        Sophus::SE3f pose = m_SLAM->TrackMonocular(m_cvImPtr->image, msg->header.stamp.sec);
        now_ = pose.inverse();
        PublishPointCloud();
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MonocularSlamNode::PublishPointCloud()
{
    ORB_SLAM3::Atlas* atlas = m_SLAM->GetAtlas();
    if (!atlas) return;

    std::vector<ORB_SLAM3::KeyFrame*> keyframes = atlas->GetAllKeyFrames();
    std::set<ORB_SLAM3::MapPoint*> all_map_points;

    for (auto kf : keyframes)
    {
        if (kf)
        {
            std::set<ORB_SLAM3::MapPoint*> mps_set = kf->GetMapPoints();
            all_map_points.insert(mps_set.begin(), mps_set.end());
        }
    }

    std::vector<ORB_SLAM3::MapPoint*> map_points(all_map_points.begin(), all_map_points.end());
    sensor_msgs::msg::PointCloud2 pointcloud_msg = ConvertToPointCloud2(map_points);
    m_pointcloud_publisher->publish(pointcloud_msg);
}

sensor_msgs::msg::PointCloud2 MonocularSlamNode::ConvertToPointCloud2(const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (const auto& mp : map_points)
    {
        if (mp && !mp->isBad())
        {
            Eigen::Vector3f pos = mp->GetWorldPos();
            cloud.push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
        }
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    return msg;
}

void MonocularSlamNode::OccupancyGridLoop()
{
    cv::Mat canvas(cell_size, cell_size, CV_8UC3, cv::Scalar(120, 120, 120));

    while (running)
    {
        for (int i = 0; i < cell_size; ++i)
            for (int j = 0; j < cell_size; ++j)
            {
                atomic_cnts[0][i][j] = 0;
                atomic_cnts[1][i][j] = 0;
            }

        canvas.setTo(cv::Scalar(120, 120, 120));

        const auto mps = m_SLAM->GetAtlas()->GetAllMapPoints();

#pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < mps.size(); ++i)
        {
            auto p = mps[i]->GetWorldPos();
            int c = cell_size / 2 + static_cast<int>(p(0) / res);
            int r = cell_size / 2 - static_cast<int>(p(2) / res);
            int z = static_cast<int>(p(1) / res);

            auto p0 = mps[i]->GetReferenceKeyFrame()->GetPose().inverse().translation();
            int c0 = cell_size / 2 + static_cast<int>(p0(0) / res);
            int r0 = cell_size / 2 - static_cast<int>(p0(2) / res);
            int z0 = static_cast<int>(p0(1) / res);

            if (std::abs(z - z0) > static_cast<int>(1.f / res)) continue;

            Bresenham(r0, c0, r, c);
        }

        DrawOccupancyMap(canvas);

        auto trans = now_.translation();
        auto dir = now_.rotationMatrix().col(2);
        int c = cell_size / 2 + static_cast<int>(trans(0) / res);
        int r = cell_size / 2 - static_cast<int>(trans(2) / res);
        int ratio[2] = {static_cast<int>(dir(0) / res), -static_cast<int>(dir(2) / res)};
        int dc = (20 * ratio[0]) / (std::abs(ratio[0]) + std::abs(ratio[1]));
        int dr = (20 * ratio[1]) / (std::abs(ratio[0]) + std::abs(ratio[1]));

        if (r >= 0 && r < cell_size && c >= 0 && c < cell_size)
        {
            cv::circle(canvas, cv::Point(c, r), 0, cv::Scalar(0, 0, 255), 10);
            cv::arrowedLine(canvas, cv::Point(c, r), cv::Point(c + dc, r + dr), cv::Scalar(0, 0, 255), 2, cv::LINE_8, 0, 0.5);
        }

        cv::imshow("Canvas", canvas);
        cv::waitKey(1);

        usleep(100 * 1000);
    }
}

bool MonocularSlamNode::check_boundary(int r, int c)
{
    return !(r < 0 || c < 0 || r >= cell_size || c >= cell_size);
}

void MonocularSlamNode::Bresenham(int r1, int c1, int r2, int c2)
{
    if (check_boundary(r2, c2)) atomic_cnts[1][r2][c2].fetch_add(1);

    if (c1 == c2)
    {
        if (r1 > r2) std::swap(r1, r2);
        while (r1 <= r2)
        {
            if (!check_boundary(r1, c1)) break;
            atomic_cnts[0][r1][c1].fetch_add(1);
            r1++;
        }
    }
    else
    {
        if (c1 > c2) { std::swap(c1, c2); std::swap(r1, r2); }
        int dr = r2 - r1, dc = c2 - c1;

        if (std::abs(dc) >= std::abs(dr))
        {
            int p = 2 * std::abs(dr) - std::abs(dc);
            int y = r1;
            for (int x = c1; x <= c2; x++)
            {
                if (!check_boundary(y, x)) break;
                atomic_cnts[0][y][x].fetch_add(1);
                if (p > 0) { y += (dr > 0) ? 1 : -1; p += 2 * (std::abs(dr) - std::abs(dc)); }
                else { p += 2 * std::abs(dr); }
            }
        }
        else
        {
            int p = 2 * std::abs(dc) - std::abs(dr);
            int x = c1;
            for (int y = r1; y <= r2; y++)
            {
                if (!check_boundary(y, x)) break;
                atomic_cnts[0][y][x].fetch_add(1);
                if (p > 0) { x += (dc > 0) ? 1 : -1; p += 2 * (std::abs(dc) - std::abs(dr)); }
                else { p += 2 * std::abs(dc); }
            }
        }
    }
}

void MonocularSlamNode::DrawOccupancyMap(cv::Mat& canvas)
{
#pragma omp parallel for collapse(2)
    for (int i = 0; i < cell_size; ++i)
    {
        for (int j = 0; j < cell_size; ++j)
        {
            int visit_cnt = 0, occupy_cnt = 0;
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    if (!check_boundary(i + dr, j + dc)) continue;
                    visit_cnt += atomic_cnts[0][i + dr][j + dc];
                    occupy_cnt += atomic_cnts[1][i + dr][j + dc];
                }
            }

            if (visit_cnt < 5)
                continue;  // Not enough visits, leave as gray

            const int percent = (occupy_cnt * 100) / visit_cnt;
            if (percent >= 15)
            {
                // Occupied → black dot
                cv::circle(canvas, cv::Point(j, i), 0, cv::Scalar(0, 0, 0), 3);
            }
            else
            {
                // Free space → white dot
                cv::circle(canvas, cv::Point(j, i), 0, cv::Scalar(255, 255, 255));
            }
        }
    }
}


