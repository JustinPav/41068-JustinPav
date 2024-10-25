#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class CylinderDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CylinderDetector node.
     * Initializes the node, subscribes to necessary topics, and sets up TF listener.
     */
    CylinderDetector() : Node("cylinder_detector"), cylinder_diameter_(0.3), circumnavigation_active_(false), move_to_point_c_(false), drive_around_cylinder_(false)
    {
        resolution_ = 0.05;
        origin_x_ = -7.0;
        origin_y_ = -10.5;

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&CylinderDetector::odomCallback, this, std::placeholders::_1));

        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/nav2/goals", 10, std::bind(&CylinderDetector::goalCallback, this, std::placeholders::_1));

        map_publisher_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/map_updates", 10);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CylinderDetector::timerCallback, this));

        // Initialize the action client for Nav2's NavigateToPose action
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        // Wait until the action server is available
        waitForActionServer();
    }

    void waitForActionServer()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            // Handle the case when the server is not available
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Action server is available");
        }
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (circumnavigation_active_)
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
        // RCLCPP_INFO(this->get_logger(), "%ld circles with diameter %f detected", circle_points.size(), cylinder_diameter_);
        if (detected_circles_.empty())
        {
            for (auto points : circle_points)
            {
                RCLCPP_INFO(this->get_logger(), "Circle detected at (%f, %f)", points.first, points.second);
                publishToOccupancyGrid(points.first, points.second);
                startCircumnavigation(points.first, points.second);
            }
            detected_circles_ = circle_points;
        }
        else
        {
            checkNewPoints(circle_points);
        }
    }

    void odomCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Save the goal
        saved_goal_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Saved goal at (%f, %f)", saved_goal_.pose.position.x, saved_goal_.pose.position.y);
    }

    void startCircumnavigation(double center_x, double center_y)
    {
        RCLCPP_INFO(this->get_logger(), "Starting circumnavigation around cylinder at (%f, %f)", center_x, center_y);

        // Cancel current navigation goal
        cancelNav2Goal();

        center_x_ = center_x;
        center_y_ = center_y;

        circumnavigation_active_ = true;
        move_to_point_c_ = true;
        drive_around_cylinder_ = false;
    }

    void timerCallback()
    {
        if (circumnavigation_active_)
        {
            double rotVel{0};
            double linearVel{0};
            if (move_to_point_c_)
            {
                std::pair<double, double> point_c = calculatePointC(center_x_, center_y_);
                typedef enum
                {
                    TURNING,
                    DRIVING,
                } vehicleStatus;
                vehicleStatus status;
                double deltaTheta = calculateDeltaTheta(point_c);
                double dist = hypot(current_pose_.position.x - point_c.first, current_pose_.position.y - point_c.second);
                if (abs(deltaTheta) > M_PI / 18)
                {
                    status = TURNING;
                }
                else
                {
                    status = DRIVING;
                }

                switch (status)
                {
                case TURNING:
                    linearVel = 0;
                    // scaler multiple to control the turning velocity
                    rotVel = 0.5 * deltaTheta;
                    if (rotVel > 0.5)
                    {
                        rotVel = 0.5;
                    }
                    if (rotVel < -0.5)
                    {
                        rotVel = -0.5;
                    }
                    break;
                case DRIVING:
                    rotVel = 0;
                    linearVel = 0.5 * abs(dist);
                    if (linearVel > 0.5)
                    {
                        linearVel = 0.5;
                    }
                    break;
                }
                geometry_msgs::msg::Twist twist_msg;
                twist_msg.linear.x = linearVel;
                twist_msg.angular.z = rotVel;
                vel_publisher_->publish(twist_msg);
                RCLCPP_INFO(this->get_logger(), "%f is distance", dist);
                if (abs(dist) < 0.1)
                {
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    vel_publisher_->publish(twist_msg);
                    move_to_point_c_ = false;
                }
            }
            else if (!move_to_point_c_ && !drive_around_cylinder_)
            {
                std::pair<double, double> center = std::make_pair(center_x_, center_y_);
                checkTangent();
                double deltaTheta = calculateDeltaTheta(center);
                RCLCPP_INFO(this->get_logger(), "%f is angle", deltaTheta * (180 / M_PI));
                if (abs(deltaTheta - M_PI / 2) < M_PI / 60)
                {
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    vel_publisher_->publish(twist_msg);
                    drive_around_cylinder_ = true;
                }
            }
            else if (drive_around_cylinder_)
            {
                geometry_msgs::msg::Twist twist_msg;
                linearVel = 0.2;          // Linear speed (m/s)
                rotVel = linearVel / 0.8; // Circumnavigation radius

                twist_msg.linear.x = linearVel;
                twist_msg.angular.z = rotVel;

                vel_publisher_->publish(twist_msg);

                // Track angle covered, complete after one circle
                double adjustment_factor = 0.6;
                current_angle_ += rotVel * 0.1 * adjustment_factor;
                RCLCPP_INFO(this->get_logger(), "Rotated %f degrees around the cylinder", current_angle_ * (180 / M_PI));
                if (current_angle_ >= 2 * M_PI)
                {
                    geometry_msgs::msg::Twist twist_msg;
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    vel_publisher_->publish(twist_msg);
                    circumnavigation_active_ = false;
                    RCLCPP_INFO(this->get_logger(), "Completed circumnavigation");
                    sendSavedGoalToNav2();
                }
            }
        }
    }

    std::pair<double, double> calculatePointC(double x, double y)
    {
        std::pair<double, double> point_c;
        double desired_dist = 0.7;
        double theta = atan2(current_pose_.position.y - y, current_pose_.position.x - x);
        point_c.first = x + desired_dist * cos(theta);
        point_c.second = y + desired_dist * sin(theta);
        return point_c;
    }

    double calculateDeltaTheta(std::pair<double, double> goal)
    {
        double deltaX = goal.first - current_pose_.position.x;
        double deltaY = goal.second - current_pose_.position.y;
        double theta{0};
        if (deltaX > 0)
        {
            theta = atan(deltaY / deltaX);
        }
        else if (deltaX < 0)
        {
            theta = atan(deltaY / deltaX);
            if (deltaY > 0)
            {
                theta = M_PI + theta;
            }
            else if (deltaY < 0)
            {
                theta = theta - M_PI;
            }
            else
            {
                theta = M_PI;
            }
        }
        else
        {
            if (deltaY > 0)
            {
                theta = M_PI / 2;
            }
            else if (deltaY < 0)
            {
                theta = -M_PI / 2;
            }
        }

        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double deltaTheta = theta - yaw;
        if (deltaTheta > M_PI)
        {
            deltaTheta -= 2 * M_PI;
        }
        if (deltaTheta < -M_PI)
        {
            deltaTheta += 2 * M_PI;
        }
        return deltaTheta;
    }

    bool checkTangent()
    {
        double bot_x = current_pose_.position.x;
        double bot_y = current_pose_.position.y;

        double target_angle = atan2(center_y_ - bot_y, center_x_ - bot_x); // Angle to the center

        // Get the TurtleBot's current yaw angle (orientation)
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Calculate the difference between the current yaw and the target angle
        double angle_diff = normalizeAngle(target_angle - yaw - M_PI / 2);
        if (fabs(angle_diff) > M_PI / 180) // If the difference is significant, keep rotating
        {
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.angular.z = (angle_diff > 0) ? 0.2 : -0.2; // Rotate to align with the target
            vel_publisher_->publish(twist_msg);
            return false; // Still turning
        }
        else
        {
            geometry_msgs::msg::Twist twist_msg;
            twist_msg.angular.z = 0.0; // Stop rotation
            vel_publisher_->publish(twist_msg);
            return true; // Finished turning
        }
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    void cancelNav2Goal()
    {
        // Check if the action client is properly initialized
        if (!action_client_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
            return;
        }

        // Wait for the action server to be ready
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }
        // Cancel all goals
        auto cancel_future = action_client_->async_cancel_all_goals();
    }

    void sendSavedGoalToNav2()
    {
        saved_goal_.header.frame_id = "map";
        saved_goal_.header.stamp = this->get_clock()->now();

        saved_goal_.pose.position.x = -0.3;
        saved_goal_.pose.position.y = -2.2;
        saved_goal_.pose.orientation.z = 1;
        saved_goal_.pose.orientation.w = 1.0;

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = saved_goal_; // Set the saved goal as the new target

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](auto)
        {
            RCLCPP_INFO(this->get_logger(), "Sent the saved goal back to Nav2");
        };

        // Send the saved goal
        action_client_->async_send_goal(goal_msg, send_goal_options);
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

    std::vector<std::pair<double, double>> detectCircleClusters(std::vector<std::pair<double, double>> points)
    {
        std::vector<std::pair<double, double>> circle_centers;

        if (points.empty())
        {
            return circle_centers;
        }

        const double cluster_tolerance = 0.1; // Max distance between consecutive points in a cluster (adjust as needed)
        const int min_points_in_cluster = 6;  // Minimum number of points to form a valid cluster

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
        const double radius_tolerance = 0.01; // Allowable deviation from the target radius
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
                    else
                    {
                        return;
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

    bool calculateCircleFromThreePoints(const std::pair<double, double> &p1, const std::pair<double, double> &p2, const std::pair<double, double> &p3, double &radius, std::pair<double, double> &center)
    {
        double tol = 0.001;
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

    void publishToOccupancyGrid(double centre_x, double centre_y)
    {
        double r = 0.5 * cylinder_diameter_;
        map_msgs::msg::OccupancyGridUpdate msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();

        // Determine the cell indices for the circle
        int value = 100; // Value for occupied
        int x_min = std::floor((centre_x - r) / resolution_);
        int x_max = std::floor((centre_x + r) / resolution_);
        int y_min = std::floor((centre_y - r) / resolution_);
        int y_max = std::floor((centre_y + r) / resolution_);

        // Prepare the data array for the update
        msg.data.reserve((x_max - x_min + 1) * (y_max - y_min + 1));
        msg.x = static_cast<int>(((centre_x - r) - origin_x_) / resolution_);
        msg.y = static_cast<int>(((centre_y - r) - origin_y_) / resolution_);
        msg.width = x_max - x_min + 1;  // Width of the update
        msg.height = y_max - y_min + 1; // Height of the update

        // Calculate the cells to update
        for (int i = x_min; i <= x_max; ++i)
        {
            for (int j = y_min; j <= y_max; ++j)
            {
                // Convert cell coordinates to world coordinates
                double cell_x = (i + 0.5) * resolution_; // Center of the cell
                double cell_y = (j + 0.5) * resolution_; // Center of the cell

                // Check if the cell is inside the circle
                if (std::pow(cell_x - centre_x, 2) + std::pow(cell_y - centre_y, 2) <= std::pow(r, 2))
                {
                    msg.data.push_back(value); // Mark as occupied
                }
                else
                {
                    msg.data.push_back(0); // Mark as free
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Publishing grid update: x_min: %d, x_max: %d, y_min: %d, y_max: %d, data size: %lu", x_min, x_max, y_min, y_max, msg.data.size());
        map_publisher_->publish(msg);
    }

    void checkNewPoints(std::vector<std::pair<double, double>> circle_points)
    {
        double tolerance_for_new_circle = 1;
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
                RCLCPP_INFO(this->get_logger(), "INJURED PERSON DETECTED AT (%f, %f)", new_point.first, new_point.second);
                detected_circles_.push_back(new_point);
                publishToOccupancyGrid(new_point.first, new_point.second);
            }
        }
    }

    // Data members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Pose current_pose_;
    const double cylinder_diameter_;

    bool circumnavigation_active_, move_to_point_c_, drive_around_cylinder_;
    double center_x_, center_y_;
    double initial_angle_, current_angle_;

    double resolution_, origin_x_, origin_y_;

    std::vector<std::pair<double, double>> detected_circles_;

    geometry_msgs::msg::PoseStamped saved_goal_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}