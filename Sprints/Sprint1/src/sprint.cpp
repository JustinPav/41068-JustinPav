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
        // republish a scan consisting of every nth point of the scanning data array
        int n = 3; // Implies we republish every 3rd scan

        // Create a new LaserScan message for the filtered data
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        std::vector<float> modified_ranges;
        for (int i = 0; i < scan->ranges.size(); i++)
        {
            if (i % n == 0)
            {
                modified_ranges.push_back(scan->ranges.at(i));
            }
        }
        new_scan->ranges = modified_ranges;
        new_scan->angle_increment = n * scan->angle_increment;

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
