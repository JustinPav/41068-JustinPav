#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MapOverlayNode : public rclcpp::Node
{
public:
    MapOverlayNode() : Node("map_overlay_node")
    {
        // Load the static map image
        const char *home = std::getenv("HOME");
        std::string map_path = std::string(home) + "/Maps/map_generator/map.pgm";
        static_map_image_ = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (static_map_image_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the static map image.");
            rclcpp::shutdown();
            return;
        }

        map_resolution_ = 0.01;
        map_origin_x_ = -5.8;
        map_origin_y_ = -11;

        // Flip the static map image vertically to match the ROS coordinate frame
        cv::flip(static_map_image_, static_map_image_, 0);

        // Convert static map to a 3-channel image
        cv::cvtColor(static_map_image_, static_map_image_color_, cv::COLOR_GRAY2BGR);

        // Create a resizable window
        cv::namedWindow("Map Overlay", cv::WINDOW_NORMAL);

        // Subscribe to the occupancy grid topic
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapOverlayNode::occupancyGridCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Map overlay node has started.");
    }

private:
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int grid_width = msg->info.width;
        int grid_height = msg->info.height;
        double grid_resolution = msg->info.resolution;
        double grid_origin_x = msg->info.origin.position.x;
        double grid_origin_y = msg->info.origin.position.y;

        // Calculate the scale factor to match the resolutions
        double scale_factor = map_resolution_ / grid_resolution;

        // Resize the static map to match the occupancy grid resolution using INTER_LINEAR for better quality
        cv::Mat resized_static_map;
        cv::resize(static_map_image_color_, resized_static_map, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);

        // Convert the occupancy grid data into an OpenCV image
        cv::Mat occupancy_grid_image(grid_height, grid_width, CV_8UC1, cv::Scalar(128)); // Default to gray for unknown

        for (int y = 0; y < grid_height; ++y)
        {
            for (int x = 0; x < grid_width; ++x)
            {
                int8_t grid_value = msg->data[y * grid_width + x];
                if (grid_value == 0)
                {
                    occupancy_grid_image.at<uchar>(y, x) = 255; // Free: White
                }
                else if (grid_value > 0)
                {
                    occupancy_grid_image.at<uchar>(y, x) = 0; // Occupied: Black
                }
            }
        }

        // Compute the offset of the occupancy grid relative to the resized static map
        int offset_x = static_cast<int>((grid_origin_x - map_origin_x_) / grid_resolution);
        int offset_y = static_cast<int>((grid_origin_y - map_origin_y_) / grid_resolution);

        // Create an overlay by cloning the resized static map
        cv::Mat overlayed_image = resized_static_map.clone();

        // Overlay the occupancy grid onto the resized static map in red
        for (int y = 0; y < grid_height; ++y)
        {
            for (int x = 0; x < grid_width; ++x)
            {
                int map_x = offset_x + x;
                int map_y = offset_y + y;
                // Ensure we're within the resized map's bounds
                if (map_x >= 0 && map_x < overlayed_image.cols && map_y >= 0 && map_y < overlayed_image.rows)
                {
                    uchar occupancy_value = occupancy_grid_image.at<uchar>(y, x);
                    if (occupancy_value == 0)
                    {
                        // Overlay red for occupied cells without changing the underlying static map details
                        overlayed_image.at<cv::Vec3b>(map_y, map_x) = cv::Vec3b(0, 0, 255); // Red color
                    }
                }
            }
        }

        // Display the overlayed image
        cv::imshow("Map Overlay", overlayed_image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    cv::Mat static_map_image_;
    cv::Mat static_map_image_color_;
    double map_resolution_;
    double map_origin_x_, map_origin_y_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOverlayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
