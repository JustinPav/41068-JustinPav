#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() : Node("cylinder_detector"), map_received_(false), cylinder_diameter_(0.3), tf_buffer_(this->get_clock()), tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CylinderDetector::mapCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&CylinderDetector::odomCallback, this, std::placeholders::_1));
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
        convertToMapFrame(circle_points);
        RCLCPP_INFO(this->get_logger(), "%ld circles with diameter %f detected", circle_points.size(), cylinder_diameter_);
        if (detected_circles_.empty())
        {
            detected_circles_ = circle_points;
        }
        else
        {
            checkNewPoints(circle_points);
        }

        RCLCPP_INFO(this->get_logger(), "Drawing %ld circles.", detected_circles_.size());
        for (auto point : detected_circles_)
        {
            drawOnMap(point.first, point.second);
        }
    }

    void checkNewPoints(std::vector<std::pair<double, double>> circle_points)
    {
        double tolerance_for_new_circle = 0.5;
        for (auto &new_point : circle_points)
        {
            bool is_new = true;

            for (const auto &existing_circle : detected_circles_)
            {
                double dist = std::hypot(new_point.first - existing_circle.first, new_point.second - existing_circle.second);

                // If the new point is within tolerance, it's not considered a new circle
                if (dist <= tolerance_for_new_circle)
                {
                    is_new = false;
                    break;
                }
            }

            // Add the new circle if no close match was found
            if (is_new)
            {
                RCLCPP_INFO(this->get_logger(), "New Circle Detected");
                detected_circles_.push_back(new_point);
            }
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
        if (cluster.size() < 5)
        {
            return; // We need at least 5 points to verify the arc condition
        }

        const double target_radius = cylinder_diameter_ / 2.0;
        const double radius_tolerance = 0.05; // Allowable deviation from the target radius
        std::vector<std::pair<double, double>> potential_centers;
        std::vector<double> radii;

        // Iterate through combinations of points in the cluster
        for (size_t i = 0; i < cluster.size() - 2; ++i)
        {
            for (size_t j = i + 1; j < cluster.size() - 1; ++j)
            {
                for (size_t k = j + 1; k < cluster.size(); ++k)
                {
                    std::pair<double, double> center;
                    double radius;

                    if (calculateCircleFromThreePoints(cluster.at(i), cluster.at(j), cluster.at(k), radius, center))
                    {
                        potential_centers.push_back(center);
                        radii.push_back(radius);
                    }
                }
            }
        }

        // Calculate the mean center and radius from all potential circles
        if (!potential_centers.empty())
        {
            double avg_x = 0.0, avg_y = 0.0, avg_radius = 0.0;
            for (size_t i = 0; i < potential_centers.size(); ++i)
            {
                avg_x += potential_centers[i].first;
                avg_y += potential_centers[i].second;
                avg_radius += radii[i];
            }
            avg_x /= potential_centers.size();
            avg_y /= potential_centers.size();
            avg_radius /= potential_centers.size();

            // Check if the average radius is within the tolerance range
            if (std::abs(avg_radius - target_radius) <= radius_tolerance)
            {
                circle_centers.push_back({avg_x, avg_y});
            }
        }
    }

    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1,
                                        const std::pair<double, double> &p2,
                                        const std::pair<double, double> &p3,
                                        double &radius,
                                        std::pair<double, double> &center)
    {
        double tol = 0.0001;
        double x1 = p1.first, y1 = p1.second;
        double x2 = p2.first, y2 = p2.second;
        double x3 = p3.first, y3 = p3.second;

        double ma = (y2 - y1) / (x2 - x1);
        double mb = (y3 - y2) / (x3 - x2);

        // Check for collinearity (parallel slopes)
        if (std::abs(ma - mb) < tol)
        {
            return false; // The points are collinear, can't form a circle
        }

        // Calculate center of the circle
        double cx = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
        double cy = -1 / ma * (cx - (x1 + x2) / 2) + (y1 + y2) / 2;

        center = {cx, cy};
        radius = std::sqrt(std::pow(cx - x1, 2) + std::pow(cy - y1, 2));

        return true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!map_received_)
        {
            return;
        }

        geometry_msgs::msg::PoseStamped pose_in_odom;
        pose_in_odom.header = msg->header;
        pose_in_odom.pose = msg->pose.pose;
        geometry_msgs::msg::PoseStamped pose_in_map;

        try
        {
            // Lookup the transform from "odom" to "map"
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);

            // Transform the pose from odom to map frame
            tf2::doTransform(pose_in_odom, pose_in_map, transform_stamped);

            current_pose_ = pose_in_map.pose;
            // Now current_pose_ contains the robot's pose in the map frame
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to map: %s", ex.what());
        }
    }

    void drawOnMap(double x, double y)
    {
        // Convert map coordinates to image coordinates
        int map_x = static_cast<int>((x - map_msg_.info.origin.position.x) / map_msg_.info.resolution);
        int map_y = static_cast<int>((y - map_msg_.info.origin.position.y) / map_msg_.info.resolution);

        // Ensure coordinates are within bounds
        if (map_x >= 0 && map_x < map_image_.cols && map_y >= 0 && map_y < map_image_.rows)
        {
            // Draw a green circle representing the detected cylinder
            cv::circle(map_image_, cv::Point(map_x, map_image_.rows - map_y), 5, cv::Scalar(0, 255, 0), -1);
        }

        // Display the updated map
        cv::imshow("Occupancy Grid with Cylinder", map_image_);
        cv::waitKey(1);
    }

    void convertToMapFrame(std::vector<std::pair<double, double>> &points)
    {
        // Transform the points to the map frame using the current robot pose
        tf2::Quaternion q(current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z, current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double robot_x = current_pose_.position.x;
        double robot_y = current_pose_.position.y;

        for (auto &point : points)
        {
            // Rotate and translate the point's local position to the map frame
            double map_x_cylinder = robot_x + point.first * cos(yaw) - point.second * sin(yaw);
            double map_y_cylinder = robot_y + point.first * sin(yaw) + point.second * cos(yaw);

            // Update the point to its transformed coordinates
            point.first = map_x_cylinder;
            point.second = map_y_cylinder;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    cv::Mat map_image_;
    bool map_received_;
    const double cylinder_diameter_;

    std::vector<std::pair<double, double>> detected_circles_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}
