#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        // Create a subscriber to the camera image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));
        // Create a publisher for the modified image
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_modified", 10);
    }

private:
    void imageCallback(const std::shared_ptr<sensor_msgs::msg::Image> msg)
    {
        // Convert the ROS message to a cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Draw a circle at the center of the image
        cv::Point center(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2);
        int radius = 100; // Radius of the circle
        cv::Scalar color(146, 255, 0); // Green color
        int thickness = 5; // Thickness of the circle border
        cv::circle(cv_ptr->image, center, radius, color, thickness);

        // Publish the modified image
        sensor_msgs::msg::Image::SharedPtr modified_image_msg = cv_ptr->toImageMsg();
        image_publisher_->publish(*modified_image_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}