#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <iostream>

class Lidar : public rclcpp::Node
{
public:
    Lidar() : Node("lidar_processor")
    {
        // Create a subscriber to the laser scan topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Lidar::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered laser scan data
        data_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
    {
        // Desired angle for range reading in degrees
        float desired_angle = 69;
        float angle_rad = desired_angle * M_PI / 180.0;

        // Normalize the desired angle
        float normalized_angle = angle_rad - scan->angle_min;
        if (normalized_angle < 0)
        {
            normalized_angle += 2 * M_PI;
        }

        // Calculate the index corresponding to the desired angle
        int angle_index = normalized_angle / scan->angle_increment;

        // Check if the index is within the valid range and log the range reading
        if (angle_index >= 0 && angle_index < scan->ranges.size())
        {
            float range_at_angle = scan->ranges.at(angle_index);
            RCLCPP_INFO(this->get_logger(), "Range at %.2f degrees: %.2f", desired_angle, range_at_angle);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Desired angle index is out of range");
        }

        // Define the range of angles for the subset of data (-45 to 45 degrees)
        float start_angle = (34 * M_PI / 180.0) - scan->angle_min;
        if (start_angle < 0)
        {
            start_angle += 2 * M_PI;
        }
        float end_angle = (120 * M_PI / 180.0) - scan->angle_min;
        if (end_angle < 0)
        {
            end_angle += 2 * M_PI;
        }
        int start_index = start_angle / scan->angle_increment;
        int end_index = end_angle / scan->angle_increment;

        // Create a new LaserScan message for the filtered data
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        std::vector<float> modified_ranges;
        for (int i = start_index; i != end_index; i++)
        {
            if (i >= scan->ranges.size())
            {
                i = 0;
            }
            modified_ranges.push_back(scan->ranges.at(i));
        }
        new_scan->ranges = modified_ranges;
        new_scan->angle_min = start_angle;
        new_scan->angle_max = end_angle;

        // Publish the filtered laser scan data
        data_publisher_->publish(*new_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr data_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
    return 0;
}
