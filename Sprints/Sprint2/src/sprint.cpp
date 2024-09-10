#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class SimpleLocalizer : public rclcpp::Node
{
public:
    SimpleLocalizer() : Node("simple_localizer"), origin_x_(-2.95), origin_y_(-2.57), resolution_(0.05), aligned_(false), map_collected_(false), angle_difference_(0.0), relative_orientation_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizer::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleLocalizer::odomCallback, this, std::placeholders::_1));
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&SimpleLocalizer::mapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Localizer Node started.");

        // const char *home = std::getenv("HOME");
        // std::string map_path = std::string(home) + "/Maps/my_map.pgm";
        // cv::Mat small_map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        // cv::resize(small_map, map_, cv::Size(500, 500), cv::INTER_LINEAR);
        // cv::imshow("Map", map_);
        // cv::waitKey(1);
        // if (map_.empty())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to load the map image!");
        // }
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (map_collected_)
        {
            laser_map_ = laserScanToMat(msg);
            cv::imshow("Laser Map", laser_map_);
            cv::waitKey(1);
            calculateYawChange();
            geometry_msgs::msg::Twist cmd;
            double angular_gain = 0.1;
            cmd.angular.z = angular_gain * angle_difference_;
            RCLCPP_INFO(this->get_logger(), "Rotating the the robot at %f rad/s", cmd.angular.z);
            cmd_publisher_->publish(cmd);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (map_collected_)
        {
            if (!aligned_)
            {
                double x = msg->pose.pose.position.x;
                double y = msg->pose.pose.position.y;
                current_pose_ = cv::Point2f(x, y);
                extractMapSection();
            }
        }
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid with dimensions: width = %d, height = %d", msg->info.width, msg->info.height);
        map_ = occupancyGridToImage(msg);
        cv::rotate(map_, map_, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::imshow("Map", map_);
        cv::waitKey(1);
        map_collected_ = true;
    }

    void extractMapSection()
    {
        double scaling_factor = 0.7; // Adjust this scaling factor as needed
        int half_width = static_cast<int>((scaling_factor * size_x_) / 2);
        int half_height = static_cast<int>((scaling_factor * size_y_) / 2);

        // Convert the robot's current pose from meters to map pixels
        int map_x = static_cast<int>((current_pose_.x - origin_x_) / resolution_);
        int map_y = static_cast<int>((current_pose_.y - origin_y_) / resolution_);

        // Calculate region of interest while ensuring valid bounds
        int x_start = std::max(0, map_x - half_width);
        int y_start = std::max(0, map_y - half_height);
        int x_end = std::min(map_.cols, map_x + half_width);
        int y_end = std::min(map_.rows, map_y + half_height);
        cv::Rect region(x_start, y_start, x_end - x_start, y_end - y_start);
        // Validate region size
        if (region.width > 0 && region.height > 0 && region.x + region.width <= map_.cols && region.y + region.height <= map_.rows)
        {

            map_edge_ = map_(region);
            cv::imshow("Edge Map", map_edge_);
            cv::waitKey(1);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Extracted region out of bounds or invalid! x_start: %d, y_start: %d, x_end: %d, y_end: %d", x_start, y_start, x_end, y_end);
        }
    }

    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &grid)
    {
        int grid_data;
        unsigned int row, col, val;

        cv::Mat m_MapBinImage;
        cv::Mat m_MapColImage;
        cv::Mat m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++)
        {
            for (col = 0; col < grid->info.width; col++)
            {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1)
                {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                }
                else
                {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }
        resolution_ = grid->info.resolution;
        origin_x_ = grid->info.origin.position.x;
        origin_y_ = grid->info.origin.position.y;
        size_x_ = grid->info.width;
        size_y_ = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                          0, 1, 0,
                          0, 0, 0);

        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);
        return m_MapColImage;
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int img_size = (size_x_ + size_y_) / 2;
        float max_range = scan->range_max;
        cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
        for (size_t i{0}; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges.at(i);
            if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max)
            {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x <= img_size && y >= 0 && y <= img_size)
                {
                    img.at<uchar>(y, x) = 255; // Set the pixel to white
                }
            }
        }
        return img;
    }

    void calculateYawChange()
    {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(laser_map_, map_edge_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try
        {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            }
            else
            {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    double origin_x_;
    double origin_y_;
    double resolution_;
    double size_x_;
    double size_y_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    cv::Mat map_, map_edge_, laser_map_;
    cv::Point2f current_pose_;
    double rotation_cmd_;

    bool aligned_;
    bool map_collected_;

    double angle_difference_;
    double relative_orientation_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLocalizer>());
    rclcpp::shutdown();
    return 0;
}