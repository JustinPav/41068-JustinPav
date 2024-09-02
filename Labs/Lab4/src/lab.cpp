#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <chrono>
#include <cmath>

class DeadReckoningNode : public rclcpp::Node
{
public:
    DeadReckoningNode() : Node("dead_reckoning_node"), gen_(rd_()), noise_dist_(0.0, 0.01)
    {
        linear_speed_ = this->declare_parameter("linear_speed", 0.2);
        angular_speed_ = this->declare_parameter("angular_speed", 0.0);
        distance_ = this->declare_parameter("distance", 2.0);
        direction_ = this->declare_parameter("direction", "forward");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_noisy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_noisy", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DeadReckoningNode::timer_callback, this));

        initialized_ = false;
        last_time_ = this->now();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            initial_pose_x_ = msg->pose.pose.position.x;
            initial_pose_y_ = msg->pose.pose.position.y;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch; // Temporary variables to hold roll and pitch
            m.getRPY(roll, pitch, initial_pose_theta_);

            current_pose_x_ = initial_pose_x_;
            current_pose_y_ = initial_pose_y_;
            current_pose_theta_ = initial_pose_theta_;

            initialized_ = true;
        }
    }

    void timer_callback()
    {
        if (!initialized_)
            return;

        auto current_time = this->now();
        auto delta_time = (current_time - last_time_).seconds();

        double delta_x = linear_speed_ * delta_time * cos(angular_speed_ * delta_time);
        double delta_y = linear_speed_ * delta_time * sin(angular_speed_ * delta_time);
        if (direction_ == "backward")
        {
            delta_x = -delta_x;
            delta_y = -delta_y;
        }

        current_pose_x_ += delta_x;
        current_pose_y_ += delta_y;
        current_pose_theta_ += angular_speed_ * delta_time;
        // write me code that wrap theta angle

        // current_pose_theta_ = normalize_angle(current_pose_theta_);

        double traveled_distance = std::hypot(current_pose_x_ - initial_pose_x_, current_pose_y_ - initial_pose_y_);

        if (traveled_distance >= distance_)
        {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        }
        else
        {
            publish_noisy_odometry(current_time);
            publish_cmd_vel();
        }

        last_time_ = current_time;
    }

    void publish_noisy_odometry(const rclcpp::Time &current_time)
    {
        current_pose_x_ = current_pose_x_ + noise_dist_(gen_);
        current_pose_y_ = current_pose_y_ + noise_dist_(gen_);
        current_pose_theta_ = current_pose_theta_ + noise_dist_(gen_);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, current_pose_theta_);

        nav_msgs::msg::Odometry odom_noisy;
        odom_noisy.header.stamp = current_time;
        odom_noisy.header.frame_id = "odom";
        odom_noisy.child_frame_id = "base_link";
        odom_noisy.pose.pose.position.x = current_pose_x_;
        odom_noisy.pose.pose.position.y = current_pose_y_;
        odom_noisy.pose.pose.orientation = tf2::toMsg(q);
        odom_noisy_pub_->publish(odom_noisy);
    }

    void publish_cmd_vel()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = (direction_ == "forward") ? linear_speed_ : -linear_speed_;
        cmd_vel.angular.z = angular_speed_;
        cmd_vel_pub_->publish(cmd_vel);
    }

    double normalize_angle(double angle)
    {
        while (angle >= 360.0)
            angle -= 360.0;
        while (angle < 0.0)
            angle += 360.0;
        return angle;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_speed_;
    double angular_speed_;
    double distance_;
    std::string direction_;

    bool initialized_;
    double initial_pose_x_, initial_pose_y_, initial_pose_theta_;
    double current_pose_x_, current_pose_y_, current_pose_theta_;
    rclcpp::Time last_time_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeadReckoningNode>());
    rclcpp::shutdown();
    return 0;
}