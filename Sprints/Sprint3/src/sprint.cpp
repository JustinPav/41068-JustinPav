#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() : Node("cylinder_detector"), map_received_(false), cylinder_diameter_(0.3)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CylinderDetector::mapCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg_ = *msg;
        map_received_ = true;
        map_image_ = occupancyGridToImage(msg);

        cv::imshow("Occupancy Grid", map_image_);
        cv::waitKey(1);
    }

    cv::Mat occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr &grid)
    {

        cv::Mat image(grid->info.height, grid->info.width, CV_8UC3);

        for (unsigned int y = 0; y < grid->info.height; y++)
        {
            for (unsigned int x = 0; x < grid->info.width; x++)
            {
                int index = x + (grid->info.height - y - 1) * grid->info.width;
                int value = grid->data[index];

                if (value == -1)
                {
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128); // Unknown
                }
                else if (value == 0)
                {
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // Free space
                }
                else
                {
                    image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Occupied
                }
            }
        }
        return image;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!map_received_)
        {
            return;
        }

        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        double angle = angle_min;

        std::vector<std::pair<double, double>> points;

        // Store the points detected by the laser scan
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isfinite(msg->ranges[i]))
            {
                double x = msg->ranges[i] * cos(angle);
                double y = msg->ranges[i] * sin(angle);
                points.push_back({x, y});
            }
            angle += angle_increment;
        }

        std::vector<std::pair<double, double>> circle_points = detectCircleClusters(points);
        RCLCPP_INFO(this->get_logger(), "%ld circles with diameter %f detected", circle_points.size(), cylinder_diameter_);

        for (auto point : circle_points)
        {
            drawOnMap(point.first, point.second);
        }
    }

    std::vector<std::pair<double, double>> detectCircleClusters(std::vector<std::pair<double, double>> points)
    {
        std::vector<std::pair<double, double>> circle_centers;

        if (points.empty())
        {
            return circle_centers;
        }

        const double cluster_tolerance = 0.1; // Max distance between consecutive points in a cluster (adjust as needed)
        const int min_points_in_cluster = 5;  // Minimum number of points to form a valid cluster

        std::vector<std::pair<double, double>> current_cluster;

        // Iterate through the points to group them into clusters
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (current_cluster.empty())
            {
                current_cluster.push_back(points[i]);
            }
            else
            {
                double distance = std::hypot(points[i].first - current_cluster.back().first, points[i].second - current_cluster.back().second);

                if (distance <= cluster_tolerance)
                {
                    current_cluster.push_back(points[i]);
                }
                else
                {
                    // Check if the cluster is valid
                    if (current_cluster.size() >= min_points_in_cluster)
                    {
                        // Check if the cluster forms an arc
                        detectArc(current_cluster, circle_centers);
                    }

                    // Start a new cluster
                    current_cluster.clear();
                    current_cluster.push_back(points[i]);
                }
            }
        }

        // Check the last cluster
        if (current_cluster.size() >= min_points_in_cluster)
        {
            detectArc(current_cluster, circle_centers);
        }

        return circle_centers;
    }

    void detectArc(const std::vector<std::pair<double, double>> &cluster, std::vector<std::pair<double, double>> &circle_centers)
    {
        const double target_radius = cylinder_diameter_ / 2.0;
        const double radius_tolerance = 0.02; // Allowable deviation from the target radius (adjust as needed)

        // Calculate the centroid of the points in the cluster
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const auto &point : cluster)
        {
            sum_x += point.first;
            sum_y += point.second;
        }

        double centroid_x = sum_x / cluster.size();
        double centroid_y = sum_y / cluster.size();

        // Calculate the average radius from the centroid
        double avg_radius = 0.0;
        for (const auto &point : cluster)
        {
            avg_radius += std::hypot(point.first - centroid_x, point.second - centroid_y);
        }
        avg_radius /= cluster.size();

        // Check if the average radius matches the target radius within the given tolerance
        if (std::abs(avg_radius - target_radius) <= radius_tolerance)
        {
            // Add the center of the detected arc to the list of detected cylinder centers
            circle_centers.push_back({centroid_x, centroid_y});
        }
    }

    void drawOnMap(double x, double y)
    {
        // Transform the coordinates (x, y) to the map's coordinate frame
        int map_x = static_cast<int>((x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
        int map_y = static_cast<int>((y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);

        // Make sure the coordinates are within bounds
        if (map_x >= 0 && map_x < map_image_.cols && map_y >= 0 && map_y < map_image_.rows)
        {
            // Draw a green circle representing the detected cylinder
            cv::circle(map_image_, cv::Point(map_x, map_image_.rows - map_y), 5, cv::Scalar(0, 255, 0), -1);
        }

        // Display the updated map
        cv::imshow("Occupancy Grid with Cylinder", map_image_);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    nav_msgs::msg::OccupancyGrid map_msg_;
    cv::Mat map_image_;
    bool map_received_;
    const double cylinder_diameter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
