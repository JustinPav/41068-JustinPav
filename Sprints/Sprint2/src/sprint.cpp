#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class SimpleLocalizer : public rclcpp::Node
{
public:
    SimpleLocalizer() : Node("simple_localizer"), angle_difference_(0.0), relative_orientation_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizer::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SimpleLocalizer::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Localizer Node started.");

        map_ = cv::imread("~/Maps/my_map.pgm", cv::IMREAD_GRAYSCALE);
        cv::imshow("Map", map_);
        cv::waitKey(1);
        if (map_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the map image!");
        }
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_map_ = laserScanToMat(msg);
        cv::imshow("Laser Map", laser_map_);
        cv::waitKey(1);
        calculateYawChange();
        geometry_msgs::msg::Twist cmd;
        double angular_gain{1.0};
        cmd.angular.z = angular_gain * angle_difference_;
        cmd_publisher_->publish(cmd);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        current_pose_ = cv::Point2f(x, y);
        extractMapSection();
    }

    void extractMapSection()
    {
        int section_size = 100;
        cv::Rect region(current_pose_.x - section_size / 2, current_pose_.y - section_size / 2, section_size, section_size);

        if (region.x >= 0 && region.y >= 0 && region.x + region.width < map_.cols && region.y + region.height < map_.rows)
        {
            cv::Mat map_section = map_(region);
            cv::Canny(map_section, map_edge_, 100, 200);
            cv::imshow("Edge Map", map_edge_);
            cv::waitKey(1);
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        int img_size = 500;
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
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(map_edge_, laser_map_, srcPoints, dstPoints);

        if (srcPoints.size() >= 3 && dstPoints.size() >= 3)
        {
            // Estimate the affine transformation between the two sets of points
            cv::Mat affine_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);

            // Extract the rotation angle (yaw) from the transformation matrix
            if (!affine_matrix.empty())
            {
                angle_difference_ = std::atan2(affine_matrix.at<double>(1, 0), affine_matrix.at<double>(0, 0));
                double angle_deg = angle_difference_ * (180.0 / M_PI);

                // Update relative orientation and log the result
                relative_orientation_ += angle_deg;
                RCLCPP_INFO(this->get_logger(), "Yaw change: %.2f degrees, Relative orientation: %.2f degrees", angle_deg, relative_orientation_);
            }
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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    cv::Mat map_, map_edge_, laser_map_;
    cv::Point2f current_pose_;
    double rotation_cmd_;

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